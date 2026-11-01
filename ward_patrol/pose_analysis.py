"""YOLOv8-Pose の出力から「姿勢」と「床からの高さ」を推定する純粋関数群.

転倒判定の考え方:
    姿勢(AI推定)  x  身体の高さ(カメラ幾何 + LiDAR)  ->  状態
    立位/座位        -                                 UPRIGHT (正常)
    臥位             床から 0.6 m 以上                 IN_BED  (正常: ベッドで休んでいる)
    臥位             床から 0.6 m 未満                 FALLEN  (異常: 床に倒れている)
    (立位に見えても体幹が床から 0.35 m 未満なら FALLEN: 頭側/足側から見た臥位)

閾値 0.6 m は、シミュレーションでの実測値 (床上の人の体幹 0.26〜0.47 m、
ベッド上 0.73〜0.83 m、マットレス面 0.5 m) の間の中央に置いた。

ROS に依存しないので単体テストできる (test/test_pose_analysis.py)。
"""
import math

import numpy as np

# COCO keypoint index
NOSE = 0
L_SHOULDER, R_SHOULDER = 5, 6
L_HIP, R_HIP = 11, 12
TORSO = (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)

UPRIGHT = 'UPRIGHT'
IN_BED = 'IN_BED'
FALLEN = 'FALLEN'

LYING_ANGLE_DEG = 50.0      # 体幹軸が鉛直からこれ以上傾いていれば臥位
LYING_ASPECT = 1.4          # キーポイントが使えない時: bbox の横/縦比
FLOOR_HEIGHT_THR = 0.6      # 臥位でこれ未満なら床上 [m]
UPRIGHT_FLOOR_THR = 0.35    # 立位に見えても体幹がこれ未満なら床上 [m]
KPT_THR = 0.5               # Ultralytics は信頼度 0.5 未満のキーポイント座標を 0 にする


def _mid(kpts, conf, a, b, thr):
    if conf[a] >= thr and conf[b] >= thr:
        return (kpts[a] + kpts[b]) / 2.0
    if conf[a] >= thr:
        return kpts[a]
    if conf[b] >= thr:
        return kpts[b]
    return None


def angle_from_vertical(vec):
    """画像上のベクトルが鉛直からどれだけ傾いているか [deg] (0..90)."""
    dx, dy = float(vec[0]), float(vec[1])
    if dx == 0.0 and dy == 0.0:
        return None
    return math.degrees(math.atan2(abs(dx), abs(dy)))


def torso_angle(kpts, conf, thr=KPT_THR):
    """肩の中点 -> 腰の中点 のベクトルの傾き [deg]. 求まらなければ None."""
    kpts = np.asarray(kpts, dtype=float)
    conf = np.asarray(conf, dtype=float)
    sh = _mid(kpts, conf, L_SHOULDER, R_SHOULDER, thr)
    hip = _mid(kpts, conf, L_HIP, R_HIP, thr)
    if sh is None or hip is None or np.linalg.norm(sh - hip) < 3.0:
        return None
    return angle_from_vertical(hip - sh)


def principal_axis_angle(kpts, conf, thr=KPT_THR, min_points=5):
    """信頼できるキーポイント全体の主軸(PCA)の傾き [deg]."""
    kpts = np.asarray(kpts, dtype=float)
    conf = np.asarray(conf, dtype=float)
    pts = kpts[conf >= thr]
    if len(pts) < min_points:
        return None
    centered = pts - pts.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    return angle_from_vertical(vt[0])


def classify_posture(kpts, conf, bbox, thr=KPT_THR):
    """姿勢を 'lying' / 'upright' に分類する.

    戻り値: (posture, angle_deg or None, method)
    体幹 -> キーポイント主軸 -> bbox 縦横比 の順にフォールバックする。
    """
    ang = torso_angle(kpts, conf, thr)
    method = 'torso'
    if ang is None:
        ang = principal_axis_angle(kpts, conf, thr)
        method = 'pca'
    if ang is not None:
        return ('lying' if ang >= LYING_ANGLE_DEG else 'upright'), ang, method
    x1, y1, x2, y2 = bbox
    aspect = (x2 - x1) / max(y2 - y1, 1e-6)
    return ('lying' if aspect >= LYING_ASPECT else 'upright'), None, 'bbox'


def body_row(kpts, conf, bbox, thr=KPT_THR):
    """身体の代表点の画像行 v (体幹キーポイントの中央値, 無ければ bbox 中心)."""
    kpts = np.asarray(kpts, dtype=float)
    conf = np.asarray(conf, dtype=float)
    rows = [kpts[i][1] for i in TORSO if conf[i] >= thr]
    if rows:
        return float(np.median(rows))
    return (bbox[1] + bbox[3]) / 2.0


def body_column(kpts, conf, bbox, thr=KPT_THR):
    """身体の代表点の画像列 u (体幹キーポイントの中央値, 無ければ bbox 中心).

    斜めに倒れた人では bbox 中心や最も近い部位(足など)と体幹の距離が大きく違うため、
    距離・方位は体幹の位置で測る。
    """
    kpts = np.asarray(kpts, dtype=float)
    conf = np.asarray(conf, dtype=float)
    cols = [kpts[i][0] for i in TORSO if conf[i] >= thr]
    if cols:
        return float(np.median(cols))
    return (bbox[0] + bbox[2]) / 2.0


def pixel_bearing(u, cx, fx):
    """画像の列 u をカメラ正面からの方位角 [rad] に変換 (左が正)."""
    return math.atan2(cx - u, fx)


def ray_depression(v, cy, fy, tilt=0.0):
    """画像の行 v を通る視線の俯角 [rad] (下向きが正). tilt はカメラの俯角."""
    return tilt + math.atan2(v - cy, fy)


def point_height(v, forward_dist, cam_height, cy, fy, tilt=0.0):
    """画像の行 v に写る点の床からの高さ [m].

    forward_dist はカメラからその点までの水平前方距離 (LiDAR などで得る)。
    """
    return cam_height - forward_dist * math.tan(ray_depression(v, cy, fy, tilt))


def ground_distance_from_row(v, cam_height, cy, fy, tilt=0.0):
    """床に接している点(画像の行 v)までの水平前方距離 [m]. 地平線より上なら None."""
    dep = ray_depression(v, cy, fy, tilt)
    if dep <= math.radians(1.0):
        return None
    return cam_height / math.tan(dep)


def lidar_distance(ranges, angle_min, angle_inc, a_left, a_right,
                   range_min=0.12, range_max=8.0, percentile=25):
    """LiDAR から方位範囲 [a_right, a_left] (rad, 左が正) 内の距離を返す.

    人物の手前側の面を拾うため下位パーセンタイルを使う。無効なら None。
    """
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return None
    ang = angle_min + angle_inc * np.arange(r.size)
    ang = (ang + math.pi) % (2 * math.pi) - math.pi
    lo, hi = min(a_left, a_right), max(a_left, a_right)
    mask = (ang >= lo) & (ang <= hi) & np.isfinite(r) & (r >= range_min) & (r <= range_max)
    if not mask.any():
        return None
    return float(np.percentile(r[mask], percentile))


def judge_state(posture, body_height, floor_thr=FLOOR_HEIGHT_THR):
    if posture != 'lying':
        # 骨格が縦に見えても体幹が床すれすれなら立位ではありえない
        # (頭側・足側から見た臥位は縦長に写るため)
        if body_height is not None and body_height < min(UPRIGHT_FLOOR_THR, floor_thr):
            return FALLEN
        return UPRIGHT
    if body_height is None:
        # 高さが不明な臥位は安全側(要確認)に倒す
        return FALLEN
    return FALLEN if body_height < floor_thr else IN_BED
