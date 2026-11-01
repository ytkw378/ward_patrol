"""見守りモニタ (ナースステーション側の画面を想定したダッシュボード).

AI の注釈付きカメラ映像・病棟マップ・病室ごとの状態・行動ログを 1 枚にまとめて
OpenCV ウィンドウに表示する。動画撮影時の「説明のわかりやすさ」のためのノード。
"""
import json
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ward_patrol import ward_map as wm

W, H = 960, 1000
FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'

# BGR
BG = (34, 30, 28)
PANEL = (52, 47, 44)
TEXT = (235, 235, 235)
SUB = (170, 170, 170)
GREEN = (90, 200, 90)
BLUE = (230, 160, 40)
RED = (60, 60, 235)
YELLOW = (40, 200, 230)
GRAY = (120, 120, 120)
STATE_COLOR = {'UPRIGHT': GREEN, 'IN_BED': BLUE, 'FALLEN': RED}
ROOM_COLOR = {'OK': GREEN, 'FALL': RED, 'CHECK': YELLOW, 'UNCHECKED': GRAY}
ROOM_TEXT = {'OK': '異常なし', 'FALL': '転倒検知', 'CHECK': '要確認', 'UNCHECKED': '未確認'}

MAP_ORIGIN = (14, 584)   # マップ描画領域の左上 (px)
MAP_SCALE = 41.0         # px / m


class Text:
    """PIL で日本語テキストを描くための遅延描画キュー."""

    def __init__(self):
        self.items = []
        self.fonts = {}

    def font(self, size, bold):
        key = (size, bold)
        if key not in self.fonts:
            self.fonts[key] = ImageFont.truetype(FONT_BOLD if bold else FONT_PATH, size)
        return self.fonts[key]

    def add(self, xy, text, size=16, color=TEXT, bold=False, anchor='la'):
        self.items.append((xy, text, size, color, bold, anchor))

    def render(self, img):
        pil = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil)
        for xy, text, size, color, bold, anchor in self.items:
            d.text(xy, text, font=self.font(size, bold), fill=color[::-1], anchor=anchor)
        self.items = []
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def to_px(x, y):
    x0 = wm.CORRIDOR_X[0]
    y1 = wm.ROOM_Y[1]
    return (int(MAP_ORIGIN[0] + (x - x0) * MAP_SCALE),
            int(MAP_ORIGIN[1] + (y1 - y) * MAP_SCALE))


class WardMonitor(Node):

    def __init__(self):
        super().__init__('ward_monitor')
        self.declare_parameter('window_x', 960)
        self.declare_parameter('window_y', 0)
        self.declare_parameter('record_path', '')
        self.camera = None
        self.status = None
        self.pose = None
        self.path = []
        self.frame = 0
        self.text = Text()
        self.writer = None
        self.create_subscription(Image, '/ward/annotated', self.on_image, 2)
        self.create_subscription(String, '/ward/status', self.on_status, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.25, self.render)
        cv2.namedWindow('Ward Patrol Monitor', cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow('Ward Patrol Monitor', self.get_parameter('window_x').value,
                       self.get_parameter('window_y').value)

    def on_image(self, msg):
        self.camera = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)
        if not self.path or math.hypot(self.path[-1][0] - p.position.x,
                                       self.path[-1][1] - p.position.y) > 0.05:
            self.path.append((p.position.x, p.position.y))

    # ------------------------------------------------------------ drawing
    def render(self):
        self.frame += 1
        img = np.full((H, W, 3), BG, np.uint8)
        st = self.status or {}
        alert = st.get('alert', False)
        self.draw_header(img, st)
        self.draw_camera(img, st, alert)
        self.draw_rooms(img, st)
        self.draw_map(img, st)
        self.draw_log(img, st)
        if alert and (self.frame // 3) % 2 == 0:
            cv2.rectangle(img, (0, 0), (W - 1, H - 1), RED, 10)
        img = self.text.render(img)
        cv2.imshow('Ward Patrol Monitor', img)
        cv2.waitKey(1)
        self.record(img)

    def draw_header(self, img, st):
        cv2.rectangle(img, (0, 0), (W, 52), PANEL, -1)
        self.text.add((14, 8), 'AI夜間巡回ロボット  見守りモニタ', 24, TEXT, True)
        phase = st.get('phase_ja', '待機中')
        room = st.get('room')
        if room:
            phase += f'  ({room}号室)'
        color = RED if st.get('alert') else YELLOW
        self.text.add((W - 14, 12), phase, 20, color, True, anchor='ra')
        self.text.add((W - 14, 36), f"経過 {st.get('elapsed', 0):.0f} s (sim)", 12, SUB,
                      anchor='ra')

    def draw_camera(self, img, st, alert):
        x0, y0 = 10, 62
        if self.camera is not None:
            cam = cv2.resize(self.camera, (640, 480))
            img[y0:y0 + 480, x0:x0 + 640] = cam
        else:
            cv2.rectangle(img, (x0, y0), (x0 + 640, y0 + 480), PANEL, -1)
            self.text.add((x0 + 320, y0 + 240), 'カメラ映像待ち', 20, SUB, anchor='mm')
        cv2.rectangle(img, (x0, y0), (x0 + 640, y0 + 480), GRAY, 1)
        self.text.add((x0 + 8, y0 + 6), 'ロボットカメラ + YOLOv8-Pose 骨格推定', 14, TEXT, True)
        if alert:
            overlay = img.copy()
            cv2.rectangle(overlay, (x0, y0 + 400), (x0 + 640, y0 + 480), RED, -1)
            cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)
            room = st.get('room') or ''
            self.text.add((x0 + 320, y0 + 440), f'⚠ {room}号室で転倒を検知  通報中', 30,
                          (255, 255, 255), True, anchor='mm')

    def draw_rooms(self, img, st):
        x0, y0, w = 662, 62, 288
        rooms = st.get('rooms', {})
        for i, room in enumerate(wm.ROOM_ORDER):
            info = rooms.get(room, {'status': 'UNCHECKED', 'summary': '未確認'})
            color = ROOM_COLOR.get(info['status'], GRAY)
            top = y0 + i * 112
            cv2.rectangle(img, (x0, top), (x0 + w, top + 102), PANEL, -1)
            cv2.rectangle(img, (x0, top), (x0 + 8, top + 102), color, -1)
            if st.get('room') == room:
                cv2.rectangle(img, (x0, top), (x0 + w, top + 102), YELLOW, 2)
            self.text.add((x0 + 22, top + 10), f'{room}号室', 22, TEXT, True)
            self.text.add((x0 + w - 12, top + 14), ROOM_TEXT.get(info['status'], ''), 18,
                          color, True, anchor='ra')
            self.text.add((x0 + 22, top + 56), info['summary'], 16, SUB)
        top = y0 + 3 * 112
        cv2.rectangle(img, (x0, top), (x0 + w, top + 144), PANEL, -1)
        self.text.add((x0 + 14, top + 8), 'AI 判定ルール', 16, TEXT, True)
        rules = [(GREEN, '立位 (UPRIGHT)', '骨格が縦向き'),
                 (BLUE, '在床 (IN_BED)', '臥位・高さ≥0.6m'),
                 (RED, '転倒 (FALLEN)', '臥位・床上<0.6m')]
        for j, (c, name, desc) in enumerate(rules):
            yy = top + 40 + j * 34
            cv2.circle(img, (x0 + 22, yy + 10), 7, c, -1)
            self.text.add((x0 + 36, yy), name, 14, TEXT, True)
            self.text.add((x0 + 36, yy + 17), desc, 12, SUB)

    def draw_map(self, img, st):
        x0, y0 = MAP_ORIGIN
        p0 = to_px(wm.CORRIDOR_X[0], wm.ROOM_Y[1])
        p1 = to_px(wm.CORRIDOR_X[1], wm.CORRIDOR_Y[0])
        cv2.rectangle(img, (p0[0] - 4, p0[1] - 24), (p1[0] + 4, p1[1] + 4), PANEL, -1)
        self.text.add((p0[0] + 2, p0[1] - 22), '病棟マップ', 14, TEXT, True)
        # 廊下と病室
        cv2.rectangle(img, to_px(wm.CORRIDOR_X[0], wm.CORRIDOR_Y[1]),
                      to_px(wm.CORRIDOR_X[1], wm.CORRIDOR_Y[0]), (70, 64, 60), -1)
        rooms = st.get('rooms', {})
        for room in wm.ROOM_ORDER:
            bx0, bx1, by0, by1 = wm.room_bounds(room)
            color = ROOM_COLOR.get(rooms.get(room, {}).get('status', 'UNCHECKED'), GRAY)
            fill = tuple(int(c * 0.25 + 60 * 0.75) for c in color)
            cv2.rectangle(img, to_px(bx0, by1), to_px(bx1, by0), fill, -1)
            cv2.rectangle(img, to_px(bx0, by1), to_px(bx1, by0), (200, 200, 200), 2)
            xc = wm.ROOMS[room]
            cv2.line(img, to_px(xc - wm.DOOR_WIDTH / 2, by0), to_px(xc + wm.DOOR_WIDTH / 2, by0),
                     (70, 64, 60), 3)
            bx, by = wm.bed_center(room)
            hx, hy = wm.BED_SIZE[0] / 2, wm.BED_SIZE[1] / 2
            cv2.rectangle(img, to_px(bx - hx, by + hy), to_px(bx + hx, by - hy),
                          (150, 150, 150), 1)
            self.text.add(to_px(bx0 + 0.15, by1 - 0.1), room, 14, TEXT, True)
            self.text.add(to_px(bx, by), 'bed', 10, SUB, anchor='mm')
        hx, hy = to_px(*wm.HOME[:2])
        cv2.rectangle(img, (hx - 12, hy - 12), (hx + 12, hy + 12), GREEN, 1)
        self.text.add((hx, hy + 18), 'NS', 11, GREEN, anchor='mm')
        # 走行軌跡
        if len(self.path) > 1:
            pts = np.array([to_px(x, y) for x, y in self.path], np.int32)
            cv2.polylines(img, [pts], False, YELLOW, 1, cv2.LINE_AA)
        # 検出人物
        for p in st.get('persons', []):
            c = STATE_COLOR.get(p['state'], GRAY)
            px = to_px(p['x'], p['y'])
            cv2.circle(img, px, 7, c, -1)
            cv2.circle(img, px, 7, (255, 255, 255), 1)
        if st.get('target'):
            tx, ty = to_px(*st['target'])
            r = 12 + (self.frame % 5) * 2
            cv2.circle(img, (tx, ty), r, RED, 2)
        # ロボット
        if self.pose is not None:
            x, y, yaw = self.pose
            c = to_px(x, y)
            cv2.circle(img, c, 8, (255, 255, 255), -1)
            tip = to_px(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
            cv2.line(img, c, tip, (255, 255, 255), 2)
            # カメラの視野 (FOV 62°)
            for s in (-1, 1):
                a = yaw + s * 0.54
                cv2.line(img, c, to_px(x + 1.6 * math.cos(a), y + 1.6 * math.sin(a)),
                         (200, 200, 120), 1, cv2.LINE_AA)
        # 凡例
        ly = p1[1] + 18
        items = [((255, 255, 255), 'ロボット'), (GREEN, '立位'), (BLUE, '在床'),
                 (RED, '転倒'), (YELLOW, '軌跡')]
        for i, (c, name) in enumerate(items):
            lx = x0 + 10 + i * 110
            cv2.circle(img, (lx, ly + 8), 6, c, -1)
            self.text.add((lx + 12, ly), name, 13, SUB)

    def draw_log(self, img, st):
        x0, y0, w, h = 662, 536, 288, 454
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), PANEL, -1)
        self.text.add((x0 + 12, y0 + 8), '行動ログ', 16, TEXT, True)
        events = st.get('events', [])
        y = y0 + 40
        for e in events[-8:]:
            color = RED if '緊急' in e['text'] or '転倒の疑い' in e['text'] else SUB
            self.text.add((x0 + 12, y), f"{e['t']:6.1f}s", 12, GRAY)
            for line in self.wrap(e['text'], 17):
                self.text.add((x0 + 64, y), line, 13, color)
                y += 19
            y += 7
        if st.get('n_alerts'):
            self.text.add((x0 + 12, y0 + h - 30), f"通報 {st['n_alerts']} 件", 16, RED, True)

    @staticmethod
    def wrap(text, n):
        return [text[i:i + n] for i in range(0, len(text), n)] or ['']

    def record(self, img):
        path = self.get_parameter('record_path').value
        if not path:
            return
        if self.writer is None:
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), 4, (W, H))
        self.writer.write(img)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WardMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
