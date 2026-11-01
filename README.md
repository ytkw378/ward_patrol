# ward_patrol — AI夜間巡回ロボット

病棟を自律巡回し、カメラ画像の **YOLOv8-Pose (骨格推定)** と **LiDAR** を組み合わせて
「ベッドで寝ている患者 (正常)」と「床に倒れている患者 (転倒)」を見分け、
転倒者を見つけたら接近・再確認して通報し、巡回レポートを自動生成する ROS 2 パッケージ。

- ROS 2 Humble / Gazebo Classic 11 / Python
- ロボット: TurtleBot3 Waffle Pi にカメラマスト (高さ 0.8 m・俯角 0.2 rad) を追加した `patrol_robot`
- AI: Ultralytics YOLOv8n-pose (COCO 学習済み, 推論のみ, CPU)

## 構成

```
ward_patrol/
├── ward_patrol/
│   ├── fall_detector.py   AI 転倒検知ノード (YOLOv8-Pose + LiDAR → 立位/在床/転倒)
│   ├── pose_analysis.py   姿勢分類・高さ推定などの純粋関数 (単体テスト対象)
│   ├── patrol_manager.py  巡回の状態機械 (移動・見回し・接近・再確認・通報・帰還)
│   ├── ward_monitor.py    ナースステーション想定の見守りモニタ (ダッシュボード表示)
│   ├── report.py          巡回レポート (Markdown) 生成
│   ├── ward_map.py        病棟の座標・シナリオ定義 (全ノード・world・launch で共有)
│   └── world_gen.py       ward_map から Gazebo world を生成
├── launch/ward_patrol.launch.py
├── worlds/hospital_ward.world   (world_gen.py で生成)
├── models/                patrol_robot, ward_person, person_standing
├── weights/yolov8n-pose.pt
├── scripts/               record_demo.sh (録画), make_video.py (提出用動画の作成)
└── test/test_pose_analysis.py
```

### ノードとトピック

| ノード | 入力 | 出力 |
|---|---|---|
| fall_detector | `/camera/image_raw`, `/camera/camera_info`, `/scan` | `/ward/detections` (JSON), `/ward/annotated` (画像) |
| patrol_manager | `/odom`, `/scan`, `/ward/detections`, `/ward/annotated`, `/ward/ack` | `/cmd_vel`, `/ward/status`, `/ward/alert` |
| ward_monitor | `/ward/annotated`, `/ward/status`, `/odom` | OpenCV ウィンドウ |

## 実行方法 (コンテナ内)

```bash
source /opt/ros/humble/setup.bash
source ~/airobot_ws/install/setup.bash        # turtlebot3_gazebo
cd /workspace/ros2_ws && colcon build --symlink-install --packages-select ward_patrol
source install/setup.bash
ros2 launch ward_patrol ward_patrol.launch.py scenario:=A
```

- `scenario`: `A` (302号室で転落) / `B` (303号室で転倒) / `C` (異常なし) / `random` (`layout_seed:=N` で再現)
- 通報中に `ros2 topic pub --once /ward/ack std_msgs/msg/Empty` で職員の確認を通知できる
- 結果は `/workspace/output/patrol_<日時>/` に `report.md`, `report.json`, `detections.jsonl`, 画像

録画と動画の作成:

```bash
bash /workspace/ros2_ws/src/ward_patrol/scripts/record_demo.sh A
python3 /workspace/ros2_ws/src/ward_patrol/scripts/make_video.py A /workspace/output/patrol_<日時>
```

単体テスト: `cd src/ward_patrol && python3 -m pytest test`

## 転倒判定

| 姿勢 (骨格) | 体幹の床からの高さ | 判定 |
|---|---|---|
| 立位 (体幹が縦) | ≥ 0.35 m | UPRIGHT (正常) |
| 臥位 (体幹が横) | ≥ 0.6 m | IN_BED (正常) |
| 臥位 | < 0.6 m | FALLEN (転倒) |
| 立位に見える | < 0.35 m | FALLEN (頭側・足側から見た臥位) |

高さは「体幹キーポイントの画像上の行」と「体幹方向の LiDAR 距離」から、
カメラの高さ・俯角を考慮したピンホールモデルで計算する (`pose_analysis.point_height`)。

## ライセンス

自作のコードは [MIT License](LICENSE) です。第三者の素材は、それぞれ元のライセンスに従います。

| 対象 | ライセンス |
|---|---|
| 自作コード (`ward_patrol/`, `launch/`, `scripts/`, `test/`, `worlds/`, `models/ward_person/` など) | MIT |
| `weights/yolov8n-pose.pt` ([Ultralytics](https://github.com/ultralytics/ultralytics)) | AGPL-3.0 |
| `models/person_standing/` ([osrf/gazebo_models](https://github.com/osrf/gazebo_models), Marina Kollmitz, MakeHuman) | CC BY 3.0 |
| `models/patrol_robot/model.sdf` (TurtleBot3 Waffle Pi, ROBOTIS を改変) | Apache-2.0 |

**注意**: 転倒検知には Ultralytics (AGPL-3.0) を使います。本ソフトウェアを Ultralytics と組み合わせて配布する場合、
またはネットワーク越しのサービスとして提供する場合は、全体として AGPL-3.0 の条件を満たす必要があります。
詳細とライセンス本文は [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) と [licenses/](licenses/) を参照してください。
