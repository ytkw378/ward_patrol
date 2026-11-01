"""夜間巡回タスクの管理ノード (状態機械).

    ナースステーション出発
      -> 各病室: 廊下 -> ドア -> 観察位置 -> 3方向を見回し (AI 検知を集計)
           転倒者あり: 接近 -> 近距離で再確認 -> 通報(アラート) -> 退室
      -> ナースステーションへ帰還 -> 巡回レポート出力

各動作はジェネレータとして書き、10 Hz のタイマで 1 ステップずつ進める。
シナリオ(誰がどこに倒れているか)は知らされておらず、検知結果だけで行動を決める。
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String

from ward_patrol import pose_analysis as pa
from ward_patrol import report
from ward_patrol import ward_map as wm

CAMERA_X = 0.04  # base_link から見たカメラの前方位置 [m] (マスト上)

PHASE_JA = {
    'INIT': '起動中',
    'MOVING': '移動中',
    'SCANNING': '病室を観察中',
    'APPROACHING': '転倒者へ接近中',
    'VERIFYING': '近距離で再確認中',
    'ALERT': '転倒を通報中',
    'RETURNING': '帰還中',
    'DONE': '巡回完了',
}
STATE_JA = {pa.UPRIGHT: '立位', pa.IN_BED: '在床', pa.FALLEN: '転倒'}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Track:
    """病室内で観測された1人の人物 (複数フレームの観測を統合)."""

    def __init__(self, x, y, state, det):
        self.x, self.y = x, y
        self.n = 0
        self.votes = {}
        self.best = det
        self.add(x, y, state, det)

    def add(self, x, y, state, det):
        self.n += 1
        k = 1.0 / self.n
        self.x += (x - self.x) * k
        self.y += (y - self.y) * k
        self.votes[state] = self.votes.get(state, 0) + 1
        if det['conf'] > self.best['conf']:
            self.best = det

    @property
    def state(self):
        return max(self.votes.items(), key=lambda kv: kv[1])[0]

    def to_dict(self):
        return {'x': round(self.x, 2), 'y': round(self.y, 2), 'state': self.state,
                'n': self.n, 'votes': self.votes,
                'height': self.best.get('height'), 'conf': self.best['conf']}


class PatrolManager(Node):

    def __init__(self):
        super().__init__('patrol_manager')
        self.declare_parameter('rooms', wm.ROOM_ORDER)
        self.declare_parameter('max_linear', 0.24)
        self.declare_parameter('max_angular', 0.9)
        self.declare_parameter('approach_distance', 1.3)
        self.declare_parameter('scan_offsets', [0.55, 0.0, -0.55])
        self.declare_parameter('frames_per_view', 3)
        self.declare_parameter('alert_hold', 6.0)
        self.declare_parameter('output_dir', '/workspace/output')
        self.declare_parameter('start_delay', 3.0)

        self.pose = None
        self.twist = (0.0, 0.0)
        self.scan = None
        self.latest_det = None
        self.latest_annotated = None
        self.collecting = False
        self.room_tracks = {}
        self.room_status = {r: {'status': 'UNCHECKED', 'summary': '未確認'}
                            for r in wm.ROOM_ORDER}
        self.alerts = []
        self.events = []
        self.snapshots = {}
        self.best_view = (-1, None)
        self.phase = 'INIT'
        self.room = None
        self.target = None
        self.acked = False
        self.t_start = None
        self.path = []

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/ward/detections', self.on_det, 10)
        self.create_subscription(Image, '/ward/annotated', self.on_annotated, 2)
        self.create_subscription(Empty, '/ward/ack', self.on_ack, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/ward/status', 10)
        self.pub_alert = self.create_publisher(String, '/ward/alert', 10)

        self.mission_gen = self.mission()
        self.create_timer(0.1, self.tick)
        self.create_timer(0.2, self.publish_status)

    # ------------------------------------------------------------ callbacks
    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)
        self.twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if not self.path or math.hypot(self.path[-1][0] - p.position.x,
                                       self.path[-1][1] - p.position.y) > 0.05:
            self.path.append((p.position.x, p.position.y))

    def on_scan(self, msg):
        self.scan = msg

    def on_ack(self, _):
        self.acked = True

    def on_det(self, msg):
        det = json.loads(msg.data)
        self.latest_det = det
        if not self.collecting or self.pose is None:
            return
        det['_used'] = []
        for p in det['persons']:
            if p['dist'] is None:
                continue
            x, y = self.person_world(p)
            if wm.room_of(x, y) != self.room and wm.room_of(x, y - 0.3) != self.room:
                continue  # 他の部屋・廊下の人物は対象外
            self.add_observation(x, y, p)
            det['_used'].append(p)
        self.frames_seen += 1
        self.log_detection(det)

    def on_annotated(self, msg):
        self.latest_annotated = msg
        # 観察中は「最も多くの人物が写ったフレーム」をこの部屋の代表画像にする
        if self.collecting and self.latest_det is not None:
            n = len(self.latest_det.get('_used', []))
            if n >= self.best_view[0]:
                self.best_view = (n, msg)

    # ------------------------------------------------------------ helpers
    def person_world(self, p):
        x, y, yaw = self.pose
        d, b = p['dist'], p['bearing']
        fx, fy = CAMERA_X + d * math.cos(b), d * math.sin(b)
        return (x + fx * math.cos(yaw) - fy * math.sin(yaw),
                y + fx * math.sin(yaw) + fy * math.cos(yaw))

    def add_observation(self, x, y, p):
        tracks = self.room_tracks.setdefault(self.room, [])
        best, best_d = None, 0.8
        for t in tracks:
            d = math.hypot(t.x - x, t.y - y)
            if d < best_d:
                best, best_d = t, d
        if best is None:
            tracks.append(Track(x, y, p['state'], p))
        else:
            best.add(x, y, p['state'], p)

    def log_detection(self, det):
        """AI の推論結果を JSON Lines で保存 (後から判定根拠を検証できるように)."""
        rec = {'t': round(self.elapsed(), 2), 'phase': self.phase, 'room': self.room,
               'robot': [round(v, 3) for v in self.pose],
               'persons': [{k: v for k, v in p.items() if k != 'kpts'}
                           for p in det['persons']],
               'used': len(det['_used'])}
        with open(os.path.join(self.ensure_outdir(), 'detections.jsonl'), 'a') as f:
            f.write(json.dumps(rec) + '\n')

    def log(self, text):
        t = self.elapsed()
        self.events.append({'t': round(t, 1), 'text': text})
        self.get_logger().info(f'[{t:6.1f}s] {text}')

    def elapsed(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            return 0.0
        return now - self.t_start

    def cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub_cmd.publish(t)

    def front_clearance(self, half_angle=0.45):
        if self.scan is None:
            return float('inf')
        r = np.asarray(self.scan.ranges, dtype=float)
        ang = wrap(self.scan.angle_min + self.scan.angle_increment * np.arange(r.size))
        m = (np.abs(ang) < half_angle) & np.isfinite(r) & (r > self.scan.range_min)
        return float(r[m].min()) if m.any() else float('inf')

    # ------------------------------------------------------------ behaviours
    def wait(self, sec):
        t_end = self.get_clock().now().nanoseconds * 1e-9 + sec
        while self.get_clock().now().nanoseconds * 1e-9 < t_end:
            self.cmd()
            yield

    def turn_to(self, yaw_goal, tol=0.03):
        wmax = self.get_parameter('max_angular').value
        while True:
            err = wrap(yaw_goal - self.pose[2])
            if abs(err) < tol:
                self.cmd()
                return
            w = max(-wmax, min(wmax, 2.0 * err))
            if abs(w) < 0.15:
                w = math.copysign(0.15, w)
            self.cmd(0.0, w)
            yield

    def go_to(self, gx, gy, tol=0.07, stop_dist=0.25):
        vmax = self.get_parameter('max_linear').value
        wmax = self.get_parameter('max_angular').value
        blocked_since = None
        while True:
            x, y, yaw = self.pose
            dist = math.hypot(gx - x, gy - y)
            if dist < tol:
                self.cmd()
                return
            err = wrap(math.atan2(gy - y, gx - x) - yaw)
            if abs(err) > 0.5:
                self.cmd(0.0, max(-wmax, min(wmax, 2.0 * err)))
                yield
                continue
            v = min(vmax, 0.8 * dist + 0.04) * (1.0 - abs(err) / 0.8)
            if self.front_clearance() < stop_dist and dist > stop_dist:
                # 障害物 (人・物) が近い: 停止して待つ
                if blocked_since is None:
                    blocked_since = self.elapsed()
                    self.log('前方に障害物を検知、一時停止')
                v = 0.0
                if self.elapsed() - blocked_since > 8.0:
                    self.log('障害物が解消しないため目標をスキップ')
                    self.cmd()
                    return
            else:
                blocked_since = None
            self.cmd(v, max(-wmax, min(wmax, 2.5 * err)))
            yield

    def follow(self, points):
        for p in points:
            yield from self.go_to(*p)

    def observe(self, n_frames, timeout=5.0):
        """静止した状態で AI 検知を n フレーム集める."""
        yield from self.wait(0.6)  # 停止直後の画像ブレ・遅延を避ける
        self.frames_seen = 0
        self.collecting = True
        t_end = self.elapsed() + timeout
        while self.frames_seen < n_frames and self.elapsed() < t_end:
            self.cmd()
            yield
        self.collecting = False

    def scan_room(self, room):
        self.phase = 'SCANNING'
        self.best_view = (-1, None)
        base = math.pi / 2
        for off in self.get_parameter('scan_offsets').value:
            yield from self.turn_to(base + off)
            yield from self.observe(self.get_parameter('frames_per_view').value)
        if self.best_view[1] is not None:
            self.snapshots[room] = self.save_image(self.best_view[1], f'room_{room}.jpg')

    def summarize_room(self, room):
        tracks = [t for t in self.room_tracks.get(room, []) if t.n >= 2]
        counts = {}
        for t in tracks:
            counts[t.state] = counts.get(t.state, 0) + 1
        fallen = [t for t in tracks if t.state == pa.FALLEN]
        if not tracks:
            summary = '人物なし'
        else:
            summary = '・'.join(f'{STATE_JA[s]}{n}名' for s, n in sorted(counts.items()))
        for t in self.room_tracks.get(room, []):
            self.get_logger().info(f'  track ({t.x:.2f}, {t.y:.2f}) n={t.n} votes={t.votes}')
        status = 'FALL' if fallen else 'OK'
        self.room_status[room] = {'status': status, 'summary': summary,
                                  'persons': [t.to_dict() for t in tracks]}
        self.log(f'{room}号室: {summary}' + (' → 転倒の疑い!' if fallen else ' → 異常なし'))
        return fallen

    def approach_and_alert(self, room, track):
        x, y, _ = self.pose
        tx, ty = track.x, track.y
        d = math.hypot(tx - x, ty - y)
        ad = self.get_parameter('approach_distance').value
        self.target = (tx, ty)
        self.phase = 'APPROACHING'
        self.log(f'転倒者 (推定位置 {tx:.1f}, {ty:.1f}) へ接近')
        if d > ad + 0.1:
            k = (d - ad) / d
            yield from self.go_to(x + (tx - x) * k, y + (ty - y) * k, stop_dist=0.35)
        yield from self.turn_to(math.atan2(ty - self.pose[1], tx - self.pose[0]))

        # 近距離で再確認 (遠距離の誤検知で通報しないため)
        self.phase = 'VERIFYING'
        self.room_tracks[room] = []
        yield from self.observe(4, timeout=6.0)
        near = [t for t in self.room_tracks.get(room, [])
                if math.hypot(t.x - tx, t.y - ty) < 1.2 and t.n >= 2]
        confirmed = any(t.votes.get(pa.FALLEN, 0) >= 2 for t in near)
        # 再確認で更新されたトラックを部屋の集計にも残す
        self.room_tracks[room] = near
        if not confirmed:
            self.log('近距離の再確認で転倒は認められず (誤検知として記録)')
            self.room_status[room]['status'] = 'CHECK'
            self.room_status[room]['summary'] += ' (再確認で否定)'
            self.target = None
            return

        self.phase = 'ALERT'
        snap = self.save_image(self.latest_annotated, f'alert_{room}.jpg')
        best = max(near, key=lambda t: t.votes.get(pa.FALLEN, 0))
        alert = {'room': room, 'x': round(best.x, 2), 'y': round(best.y, 2),
                 'height': best.best.get('height'), 'conf': best.best['conf'],
                 't': round(self.elapsed(), 1), 'snapshot': snap,
                 'wall_time': time.strftime('%Y-%m-%d %H:%M:%S')}
        self.alerts.append(alert)
        self.pub_alert.publish(String(data=json.dumps(alert, ensure_ascii=False)))
        self.log(f'【緊急】{room}号室で転倒を確認。ナースステーションへ通報')
        self.acked = False
        hold_end = self.elapsed() + self.get_parameter('alert_hold').value
        while not self.acked and self.elapsed() < hold_end:
            self.cmd()
            yield
        self.log('通報完了 (職員の対応待ち) → 巡回を継続' if not self.acked
                 else '職員が通報を確認 → 巡回を継続')
        self.target = None

    def mission(self):
        while self.pose is None or self.scan is None or self.latest_det is None:
            self.phase = 'INIT'
            yield
        self.t_start = self.get_clock().now().nanoseconds * 1e-9
        yield from self.wait(self.get_parameter('start_delay').value)
        self.log('夜間巡回を開始')
        for room in self.get_parameter('rooms').value:
            self.room = room
            wp = wm.room_waypoints(room)
            self.phase = 'MOVING'
            self.log(f'{room}号室へ移動')
            yield from self.follow([wp['corridor'], wp['door'], wp['inspect']])
            yield from self.scan_room(room)
            fallen = self.summarize_room(room)
            if fallen:
                track = max(fallen, key=lambda t: t.votes.get(pa.FALLEN, 0))
                yield from self.approach_and_alert(room, track)
                self.phase = 'MOVING'
                yield from self.go_to(*wp['inspect'], stop_dist=0.2)
            self.phase = 'MOVING'
            yield from self.follow([wp['door'], wp['corridor']])
        self.room = None
        self.phase = 'RETURNING'
        self.log('ナースステーションへ帰還')
        yield from self.go_to(wm.HOME[0], wm.HOME[1])
        yield from self.turn_to(wm.HOME[2])
        path = self.write_report()
        self.phase = 'DONE'
        self.log(f'巡回完了。レポート: {path}')
        while True:
            self.cmd()
            yield

    def tick(self):
        if self.pose is None:
            return
        try:
            next(self.mission_gen)
        except StopIteration:
            pass

    # ------------------------------------------------------------ outputs
    def publish_status(self):
        persons = []
        for room, tracks in self.room_tracks.items():
            for t in tracks:
                if t.n >= 2:
                    persons.append({'room': room, 'x': round(t.x, 2), 'y': round(t.y, 2),
                                    'state': t.state})
        status = {
            'phase': self.phase, 'phase_ja': PHASE_JA[self.phase], 'room': self.room,
            'elapsed': round(self.elapsed(), 1),
            'robot': None if self.pose is None else [round(v, 3) for v in self.pose],
            'rooms': {r: {'status': s['status'], 'summary': s['summary']}
                      for r, s in self.room_status.items()},
            'persons': persons, 'target': self.target,
            'alert': self.phase == 'ALERT', 'n_alerts': len(self.alerts),
            'events': self.events[-8:],
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def ensure_outdir(self):
        if not hasattr(self, 'outdir'):
            stamp = time.strftime('%Y%m%d_%H%M%S')
            self.outdir = os.path.join(self.get_parameter('output_dir').value, f'patrol_{stamp}')
            os.makedirs(self.outdir, exist_ok=True)
        return self.outdir

    def save_image(self, msg, name):
        if msg is None:
            return None
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        path = os.path.join(self.ensure_outdir(), name)
        cv2.imwrite(path, img)
        return name

    def write_report(self):
        outdir = self.ensure_outdir()
        data = {
            'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration_sec': round(self.elapsed(), 1),
            'rooms': self.room_status, 'alerts': self.alerts,
            'events': self.events, 'snapshots': self.snapshots,
            'path': [[round(x, 2), round(y, 2)] for x, y in self.path[::5]],
        }
        with open(os.path.join(outdir, 'report.json'), 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        md = os.path.join(outdir, 'report.md')
        with open(md, 'w') as f:
            f.write(report.render_markdown(data))
        return md


def main():
    rclpy.init()
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
