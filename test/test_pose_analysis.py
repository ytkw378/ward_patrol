import math

import numpy as np

from ward_patrol import pose_analysis as pa
from ward_patrol import ward_map as wm


def skeleton(shoulder, hip, conf=0.9):
    """肩と腰の位置だけを持つ 17 点のキーポイントを作る."""
    k = np.zeros((17, 2))
    c = np.zeros(17)
    k[pa.L_SHOULDER] = (shoulder[0] - 10, shoulder[1])
    k[pa.R_SHOULDER] = (shoulder[0] + 10, shoulder[1])
    k[pa.L_HIP] = (hip[0] - 8, hip[1])
    k[pa.R_HIP] = (hip[0] + 8, hip[1])
    c[list(pa.TORSO)] = conf
    return k, c


def test_upright_and_lying_from_torso():
    k, c = skeleton((100, 50), (102, 150))
    assert pa.classify_posture(k, c, (80, 20, 120, 250))[0] == 'upright'
    k, c = skeleton((50, 200), (160, 210))
    posture, ang, method = pa.classify_posture(k, c, (30, 180, 260, 230))
    assert posture == 'lying' and ang > 80 and method == 'torso'


def test_bbox_fallback_when_no_keypoints():
    k, c = np.zeros((17, 2)), np.zeros(17)
    assert pa.classify_posture(k, c, (0, 0, 300, 60)) == ('lying', None, 'bbox')
    assert pa.classify_posture(k, c, (0, 0, 60, 300)) == ('upright', None, 'bbox')


def test_geometry_round_trip():
    fy, cy, cam_h = 530.0, 240.5, 0.10
    # 前方 2 m・高さ 0.6 m の点が写る行を求め、逆算して戻ることを確認
    v = cy - (0.6 - cam_h) * fy / 2.0
    assert math.isclose(pa.point_height(v, 2.0, cam_h, cy, fy), 0.6, abs_tol=1e-9)
    v_floor = cy + cam_h * fy / 3.0
    assert math.isclose(pa.ground_distance_from_row(v_floor, cam_h, cy, fy), 3.0, rel_tol=1e-9)
    assert pa.ground_distance_from_row(cy - 5, cam_h, cy, fy) is None


def test_geometry_with_tilted_camera():
    fy, cy, cam_h, tilt = 530.0, 240.5, 0.81, 0.20
    # 前方 2.5 m・高さ 0.6 m の点 -> 行 v -> 高さを逆算
    dep = math.atan2(cam_h - 0.6, 2.5)
    v = cy + fy * math.tan(dep - tilt)
    assert math.isclose(pa.point_height(v, 2.5, cam_h, cy, fy, tilt), 0.6, abs_tol=1e-9)
    # 床上 1.8 m の点
    v_floor = cy + fy * math.tan(math.atan2(cam_h, 1.8) - tilt)
    assert math.isclose(pa.ground_distance_from_row(v_floor, cam_h, cy, fy, tilt), 1.8,
                        rel_tol=1e-9)
    # 俯角があるので画像中心より少し上でも床は見える
    assert pa.ground_distance_from_row(cy - 20, cam_h, cy, fy, tilt) is not None


def test_bearing_sign_is_left_positive():
    assert pa.pixel_bearing(100, 320, 530) > 0
    assert pa.pixel_bearing(500, 320, 530) < 0


def test_lidar_distance_picks_sector():
    ranges = np.full(360, 5.0)
    ranges[355:] = 1.5   # 右前方
    ranges[:6] = 1.5     # 左前方
    ranges[90] = 0.5     # 真横 (範囲外)
    d = pa.lidar_distance(ranges, 0.0, math.radians(1), math.radians(4), math.radians(-4))
    assert math.isclose(d, 1.5)
    assert pa.lidar_distance([float('inf')] * 360, 0.0, math.radians(1), 0.1, -0.1) is None


def test_judge_state():
    assert pa.judge_state('upright', 1.1) == pa.UPRIGHT
    assert pa.judge_state('upright', 0.1) == pa.FALLEN   # 足側から見た臥位
    assert pa.judge_state('lying', 0.62) == pa.IN_BED
    assert pa.judge_state('lying', 0.15) == pa.FALLEN
    assert pa.judge_state('lying', 0.47) == pa.FALLEN    # 体の厚み分を含んでも床上
    assert pa.judge_state('upright', 0.45) == pa.UPRIGHT  # 床に座っている程度は立位扱い
    assert pa.judge_state('lying', None) == pa.FALLEN


def test_scenarios_place_people_inside_rooms():
    for name in list(wm.SCENARIOS) + ['random']:
        for seed in range(5):
            people = wm.scenario_people(name, seed)
            assert sum(k == 'fallen' for _, k, _, _ in people) <= 1
            for _, _, room, (x, y, *_rest) in people:
                assert wm.room_of(x, y) == room
