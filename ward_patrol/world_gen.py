"""ward_map の定義から病棟の Gazebo world (SDF) を生成する.

    python3 -m ward_patrol.world_gen > worlds/hospital_ward.world
"""
from ward_patrol import ward_map as wm

WALL_T = 0.1
WALL_H = 1.6  # マストのカメラ(0.8 m)から隣室の人が見えない高さ

WALL_COLOR = '0.93 0.91 0.85 1'
FLOOR_COLOR = '0.78 0.84 0.86 1'
BED_COLOR = '0.97 0.97 1.0 1'
SHEET_COLOR = '0.62 0.78 0.92 1'
DESK_COLOR = '0.55 0.42 0.30 1'


def _box(name, x, y, z, sx, sy, sz, color, collide=True):
    coll = (f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size>'
            f'</box></geometry></collision>') if collide else ''
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        {coll}
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>'''


def _wall_x(name, x0, x1, y):
    """x 方向に伸びる壁."""
    return _box(name, (x0 + x1) / 2, y, WALL_H / 2, x1 - x0, WALL_T, WALL_H, WALL_COLOR)


def _wall_y(name, y0, y1, x):
    """y 方向に伸びる壁."""
    return _box(name, x, (y0 + y1) / 2, WALL_H / 2, WALL_T, y1 - y0, WALL_H, WALL_COLOR)


def walls():
    cx0, cx1 = wm.CORRIDOR_X
    cy0, cy1 = wm.CORRIDOR_Y
    ry0, ry1 = wm.ROOM_Y
    rooms_x0 = wm.ROOMS[wm.ROOM_ORDER[0]] - wm.ROOM_HALF_WIDTH
    rooms_x1 = wm.ROOMS[wm.ROOM_ORDER[-1]] + wm.ROOM_HALF_WIDTH
    out = [
        _wall_x('wall_south', cx0, cx1, cy0),
        _wall_y('wall_west', cy0, cy1, cx0),
        _wall_y('wall_east', cy0, cy1, cx1),
        _wall_x('wall_ns_north', cx0, rooms_x0, cy1),
        _wall_x('wall_east_north', rooms_x1, cx1, cy1),
        _wall_x('wall_rooms_north', rooms_x0, rooms_x1, ry1),
    ]
    for i, room in enumerate(wm.ROOM_ORDER):
        x0, x1, _, _ = wm.room_bounds(room)
        xc = wm.ROOMS[room]
        half = wm.DOOR_WIDTH / 2
        out.append(_wall_x(f'wall_{room}_s1', x0, xc - half, ry0))
        out.append(_wall_x(f'wall_{room}_s2', xc + half, x1, ry0))
        if i == 0:
            out.append(_wall_y(f'wall_{room}_w', ry0, ry1, x0))
        out.append(_wall_y(f'wall_{room}_e', ry0, ry1, x1))
    return ''.join(out)


def beds():
    out = []
    sx, sy, sz = wm.BED_SIZE
    for room in wm.ROOM_ORDER:
        bx, by = wm.bed_center(room)
        out.append(_box(f'bed_{room}', bx, by, (sz - 0.08) / 2, sx, sy, sz - 0.08, BED_COLOR))
        # シーツ(薄い板)・枕 (頭側 = 東)・床頭台
        out.append(_box(f'sheet_{room}', bx - 0.15, by, sz - 0.04, sx - 0.3, sy - 0.04, 0.08,
                        SHEET_COLOR))
        out.append(_box(f'pillow_{room}', bx + sx / 2 - 0.2, by, sz + 0.0, 0.3, 0.6, 0.1,
                        BED_COLOR, collide=False))
        out.append(_box(f'cabinet_{room}', bx + 0.7, by + sy / 2 + 0.35, 0.35, 0.45, 0.45, 0.7,
                        DESK_COLOR))
    return ''.join(out)


def nurse_station():
    return (_box('ns_desk', -1.0, -1.0, 0.45, 1.6, 0.6, 0.9, DESK_COLOR)
            + _box('ns_dock', 0.0, 0.0, 0.005, 0.6, 0.6, 0.01, '0.2 0.6 0.3 1', collide=False))


def world_sdf():
    floor = _box('ward_floor', 5.75, 2.25, -0.005, 15.5, 7.6, 0.01, FLOOR_COLOR, collide=False)
    hx, hy, hyaw = wm.HOME
    cam_back, cam_z = -1.8, 7.0
    return f'''<?xml version="1.0"?>
<!-- このファイルは ward_patrol/world_gen.py で生成しています。直接編集しないでください。 -->
<sdf version="1.6">
  <world name="hospital_ward">
    <include><uri>model://ground_plane</uri></include>
    <light name="ceiling" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.4 -1.0</direction>
    </light>
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    <gui fullscreen="0">
      <!-- 動画用: ロボットの後方上空から追従するチェイスカメラ (壁 1.6 m に遮られない角度) -->
      <camera name="user_camera">
        <pose>{hx + cam_back:.2f} {hy:.2f} {cam_z:.2f} 0 1.3 0</pose>
        <track_visual>
          <name>patrol_robot</name>
          <static>true</static>
          <use_model_frame>true</use_model_frame>
          <xyz>{cam_back:.2f} 0 {cam_z:.2f}</xyz>
          <inherit_yaw>true</inherit_yaw>
        </track_visual>
      </camera>
    </gui>
    {floor}
    {walls()}
    {beds()}
    {nurse_station()}
    <include>
      <uri>model://patrol_robot</uri>
      <name>patrol_robot</name>
      <pose>{hx:.2f} {hy:.2f} 0.01 0 0 {hyaw:.4f}</pose>
    </include>
  </world>
</sdf>
'''


def main():
    print(world_sdf())


if __name__ == '__main__':
    main()
