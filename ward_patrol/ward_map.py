"""病棟の幾何情報とシナリオ定義.

world ファイル(worlds/hospital_ward.world)・巡回ノード・モニタノード・launch が
同じ座標系を共有するための唯一の定義元。座標はすべて Gazebo のワールド座標[m]。

    y
    ^   +-------+-------+-------+
    |   |  301  |  302  |  303  |   病室 (y = 1.5 .. 6.0)
    |   |   bed=|   bed=|   bed=|   ベッドは東の壁に頭側をつけて横向きに置く
    |   +--D----+--D----+--D----+   D: ドア (幅 1.2 m)
    |  NS          廊下              (y = -1.5 .. 1.5)
    +---------------------------------> x
       NS: ナースステーション(ロボットの待機位置 = 原点)
"""
import math
import random

CORRIDOR_Y = (-1.5, 1.5)
CORRIDOR_X = (-2.0, 13.5)
ROOM_Y = (1.5, 6.0)
DOOR_WIDTH = 1.2
BED_SIZE = (2.0, 1.0, 0.5)  # x(長さ), y(幅), 高さ

HOME = (0.0, 0.0, 0.0)  # ナースステーション (x, y, yaw)

# 病室: 名前 -> 中心 x 座標. 部屋の幅は 4 m.
ROOMS = {
    '301': 3.0,
    '302': 7.0,
    '303': 11.0,
}
ROOM_ORDER = ['301', '302', '303']
ROOM_HALF_WIDTH = 2.0


def room_bounds(room):
    xc = ROOMS[room]
    return (xc - ROOM_HALF_WIDTH, xc + ROOM_HALF_WIDTH, ROOM_Y[0], ROOM_Y[1])


def bed_center(room):
    """ベッド中心. 頭側を東の壁につけ、入口から側面が見える向きに置く."""
    return (ROOMS[room] + ROOM_HALF_WIDTH - 0.05 - BED_SIZE[0] / 2, 4.4)


def room_waypoints(room):
    """病室を見回るための経由点: 廊下 -> ドア -> 観察位置."""
    xc = ROOMS[room]
    return {
        'corridor': (xc, 0.0),
        'door': (xc, ROOM_Y[0]),
        'inspect': (xc, ROOM_Y[0] + 0.5),
    }


def room_of(x, y):
    for name in ROOM_ORDER:
        x0, x1, y0, y1 = room_bounds(name)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


# ---------------------------------------------------------------- 人物配置
# 人物モデル(person_standing のメッシュ)は足元が原点, 立位で正面が -y.
# 臥位は roll=-pi/2 で頭が +y 方向を向き、さらに yaw=psi で頭の向きは (-sin psi, cos psi)。
_LYING_Z = 0.15  # 臥位時の体の厚みの半分


def person_pose(kind, room, variant=0):
    """人物の種類と病室から spawn 姿勢 (x, y, z, roll, pitch, yaw) を返す."""
    xc = ROOMS[room]
    bx, by = bed_center(room)
    if kind == 'bed':  # ベッド上で臥床している患者 (正常). 頭は東 (+x)
        return (bx - 0.9, by, BED_SIZE[2] + _LYING_Z, -math.pi / 2, 0.0, -math.pi / 2)
    if kind == 'fallen':  # ベッドから転落し床に倒れている患者 (異常). 向きは3通り
        feet_x, feet_y, yaw = [(-0.3, 3.5, 1.25), (-1.4, 3.1, -0.3), (-1.7, 4.9, -2.3)][variant % 3]
        return (xc + feet_x, feet_y, _LYING_Z, -math.pi / 2, 0.0, yaw)
    if kind == 'standing':  # 立っている職員・見舞い客 (正常)
        return (xc - 1.0, 4.6, 0.0, 0.0, 0.0, 0.3)
    raise ValueError(kind)


SCENARIOS = {
    # 302 号室の患者がベッドから転落
    'A': {'301': ['bed'], '302': ['fallen'], '303': ['bed', 'standing']},
    # 303 号室で転倒。301 には職員が訪室中
    'B': {'301': ['bed', 'standing'], '302': ['bed'], '303': ['fallen']},
    # 異常なし (誤報が出ないことの確認)
    'C': {'301': ['bed'], '302': ['bed', 'standing'], '303': ['bed']},
}


def scenario_people(name, seed=None):
    """シナリオ名から [(entity名, kind, room, pose), ...] を返す.

    'random' の場合は転倒者のいる病室と職員の位置をランダムに決める。
    """
    rng = random.Random(seed)
    if name == 'random':
        fallen = rng.choice(ROOM_ORDER)
        layout = {}
        for room in ROOM_ORDER:
            layout[room] = ['fallen'] if room == fallen else ['bed']
            if room != fallen and rng.random() < 0.5:
                layout[room].append('standing')
    else:
        layout = SCENARIOS[name]
    people = []
    for room in ROOM_ORDER:
        for kind in layout.get(room, []):
            pose = person_pose(kind, room, variant=rng.randrange(3))
            people.append((f'person_{kind}_{room}', kind, room, pose))
    return people
