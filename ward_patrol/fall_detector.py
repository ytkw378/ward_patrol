"""AI 転倒検知ノード.

カメラ画像に YOLOv8-Pose (骨格推定) をかけ、人物ごとに
  1. 骨格から姿勢 (立位 / 臥位) を分類
  2. LiDAR から人物までの距離を取得 (カメラと LiDAR のセンサフュージョン)
  3. 距離と画像上の位置から身体の床からの高さを計算
  4. 姿勢 x 高さ で UPRIGHT / IN_BED / FALLEN を判定
して /ward/detections (JSON) と注釈付き画像 /ward/annotated を出力する。
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from ward_patrol import pose_analysis as pa

STATE_COLORS = {  # BGR
    pa.UPRIGHT: (80, 200, 80),
    pa.IN_BED: (230, 160, 40),
    pa.FALLEN: (40, 40, 240),
}
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6)]


def imgmsg_to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    return a.copy()


def bgr_to_imgmsg(img, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = img.shape[1] * 3
    msg.data = img.tobytes()
    return msg


class FallDetector(Node):

    def __init__(self):
        super().__init__('fall_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('keypoint_threshold', pa.KPT_THR)
        # カメラはマスト上 (patrol_robot/model.sdf と一致させる)
        self.declare_parameter('camera_height', 0.81)
        self.declare_parameter('camera_tilt', 0.20)
        self.declare_parameter('lidar_behind_camera', 0.104)
        self.declare_parameter('floor_height_threshold', pa.FLOOR_HEIGHT_THR)
        self.declare_parameter('rate', 4.0)
        self.declare_parameter('image_size', 480)
        self.declare_parameter('torch_threads', 2)

        import torch
        from ultralytics import YOLO
        torch.set_num_threads(self.get_parameter('torch_threads').value)
        model_path = self.get_parameter('model_path').value or self._default_model()
        self.get_logger().info(f'loading YOLOv8-Pose: {model_path}')
        self.model = YOLO(model_path)

        self.image = None
        self.image_new = False
        self.info = None
        self.scan = None

        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(String, '/ward/detections', 10)
        self.pub_img = self.create_publisher(Image, '/ward/annotated', 2)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.process)

    def _default_model(self):
        path = os.path.join(get_package_share_directory('ward_patrol'),
                            'weights', 'yolov8n-pose.pt')
        return path if os.path.exists(path) else 'yolov8n-pose.pt'

    def on_image(self, msg):
        self.image = msg
        self.image_new = True

    def on_info(self, msg):
        self.info = msg

    def on_scan(self, msg):
        self.scan = msg

    # ------------------------------------------------------------------
    def process(self):
        if not self.image_new or self.info is None:
            return
        self.image_new = False
        msg = self.image
        img = imgmsg_to_bgr(msg)
        fx, cx = self.info.k[0], self.info.k[2]
        fy, cy = self.info.k[4], self.info.k[5]
        cam_h = self.get_parameter('camera_height').value
        tilt = self.get_parameter('camera_tilt').value
        kthr = self.get_parameter('keypoint_threshold').value
        floor_thr = self.get_parameter('floor_height_threshold').value

        res = self.model(img, conf=self.get_parameter('conf_threshold').value,
                         imgsz=self.get_parameter('image_size').value, verbose=False)[0]
        persons = []
        if res.keypoints is not None and len(res.boxes) > 0:
            boxes = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            kxy = res.keypoints.xy.cpu().numpy()
            kcf = (res.keypoints.conf.cpu().numpy() if res.keypoints.conf is not None
                   else np.ones(kxy.shape[:2]))
            for box, conf, kp, kc in zip(boxes, confs, kxy, kcf):
                persons.append(self.analyze(box, float(conf), kp, kc, fx, cx, fy, cy,
                                            cam_h, tilt, kthr, floor_thr))
        # 骨格の根拠がない (bbox のみの) 低信頼度の検出は誤検出として捨てる
        persons = [p for p in persons if p['method'] != 'bbox' or p['conf'] >= 0.4]
        persons = self.suppress_duplicates(persons)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub_det.publish(String(data=json.dumps({'stamp': stamp, 'persons': persons})))
        self.pub_img.publish(bgr_to_imgmsg(self.draw(img, persons), msg.header))

    def analyze(self, box, conf, kp, kc, fx, cx, fy, cy, cam_h, tilt, kthr, floor_thr):
        posture, angle, method = pa.classify_posture(kp, kc, box, kthr)
        x1, y1, x2, y2 = [float(v) for v in box]
        w = x2 - x1
        # 方位・距離は体幹の位置で測る (斜めに倒れた人で足先の距離を拾わないため)
        bearing = pa.pixel_bearing(pa.body_column(kp, kc, box, kthr), cx, fx)

        dist, src = None, 'none'
        if self.scan is not None:
            r = None
            win = math.radians(3.0)
            sectors = [(bearing + win, bearing - win),
                       (pa.pixel_bearing(x1 + 0.2 * w, cx, fx),
                        pa.pixel_bearing(x2 - 0.2 * w, cx, fx))]
            for a_l, a_r in sectors:  # 体幹方向 ±3°、だめなら bbox の内側 60%
                r = pa.lidar_distance(self.scan.ranges, self.scan.angle_min,
                                      self.scan.angle_increment, a_l, a_r,
                                      self.scan.range_min, min(self.scan.range_max, 8.0),
                                      percentile=50)
                if r is not None:
                    break
            if r is not None:
                dist = max(r - self.get_parameter('lidar_behind_camera').value
                           * math.cos(bearing), 0.05)
                src = 'lidar'
        if dist is None:
            # 床に接している前提で bbox 下端から距離を推定
            g = pa.ground_distance_from_row(y2, cam_h, cy, fy, tilt)
            if g is not None:
                dist, src = g / max(math.cos(bearing), 0.2), 'ground'

        height = None
        if dist is not None:
            v = pa.body_row(kp, kc, box, kthr)
            height = pa.point_height(v, dist * math.cos(bearing), cam_h, cy, fy, tilt)
        state = pa.judge_state(posture, height, floor_thr)
        return {
            'state': state, 'posture': posture,
            'angle': None if angle is None else round(angle, 1), 'method': method,
            'conf': round(conf, 3), 'bearing': round(bearing, 4),
            'dist': None if dist is None else round(dist, 3), 'dist_src': src,
            'height': None if height is None else round(height, 3),
            'bbox': [round(x1), round(y1), round(x2), round(y2)],
            'kpts': [[round(float(p[0]), 1), round(float(p[1]), 1), round(float(c), 2)]
                     for p, c in zip(kp, kc)],
        }

    @staticmethod
    def suppress_duplicates(persons):
        """同一人物への重複検出 (bbox が大きく重なるもの) を信頼度の高い方に統合."""
        def iou_min(a, b):
            ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = ix * iy
            area = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
            return inter / max(area, 1)
        kept = []
        for p in sorted(persons, key=lambda p: -p['conf']):
            if all(iou_min(p['bbox'], k['bbox']) < 0.6 for k in kept):
                kept.append(p)
        return kept

    @staticmethod
    def draw(img, persons):
        out = img.copy()
        for p in persons:
            color = STATE_COLORS[p['state']]
            x1, y1, x2, y2 = p['bbox']
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            kp = p['kpts']
            for a, b in SKELETON:
                if kp[a][2] > pa.KPT_THR and kp[b][2] > pa.KPT_THR:
                    cv2.line(out, (int(kp[a][0]), int(kp[a][1])),
                             (int(kp[b][0]), int(kp[b][1])), color, 2)
            for x, y, c in kp:
                if c > pa.KPT_THR:
                    cv2.circle(out, (int(x), int(y)), 3, (255, 255, 255), -1)
            h = '?' if p['height'] is None else f"{p['height']:.2f}m"
            d = '?' if p['dist'] is None else f"{p['dist']:.1f}m"
            label = f"{p['state']} h={h} d={d}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(y1 - 6, th + 4)
            cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 4, ty + 3), color, -1)
            cv2.putText(out, label, (x1 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
        return out


def main():
    rclpy.init()
    node = FallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
