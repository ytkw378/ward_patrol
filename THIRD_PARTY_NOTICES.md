# Third-Party Notices

このリポジトリの自作コードは MIT License です ([LICENSE](LICENSE))。
以下の第三者の素材は、それぞれ元のライセンスに従います。ライセンス本文は [licenses/](licenses/) にあります。

| 対象 | 出典 | ライセンス | 本文 |
|---|---|---|---|
| `weights/yolov8n-pose.pt` | [Ultralytics](https://github.com/ultralytics/ultralytics) | AGPL-3.0 | [licenses/AGPL-3.0.txt](licenses/AGPL-3.0.txt) |
| `models/person_standing/` | [osrf/gazebo_models](https://github.com/osrf/gazebo_models) (Marina Kollmitz, MakeHuman) | CC BY 3.0 | [licenses/CC-BY-3.0_gazebo_models.txt](licenses/CC-BY-3.0_gazebo_models.txt) |
| `models/patrol_robot/model.sdf` | [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) Waffle Pi (ROBOTIS) を改変 | Apache-2.0 | [licenses/Apache-2.0.txt](licenses/Apache-2.0.txt) |

## 実行時に使うライブラリ (同梱していないもの)

| ライブラリ | ライセンス |
|---|---|
| Ultralytics (`ultralytics` Python パッケージ) | AGPL-3.0 |
| ROS 2 Humble, gazebo_ros, turtlebot3_gazebo | Apache-2.0 |
| Gazebo Classic 11 | Apache-2.0 |
| OpenCV | Apache-2.0 |
| NumPy | BSD-3-Clause |
| Pillow | MIT-CMU (HPND) |
| PyTorch | BSD-3-Clause |

## AGPL-3.0 についての注意

`fall_detector` は実行時に Ultralytics をインポートし、AGPL-3.0 の学習済み重みを使います。
このため、本ソフトウェアを Ultralytics と組み合わせて配布する場合、またはネットワーク越しのサービスとして
提供する場合は、組み合わせた全体について AGPL-3.0 の条件 (ソースコードの公開など) を満たす必要があります。
MIT License の自作コードは AGPL-3.0 と両立するため、この条件と矛盾しません。

## 改変の表示 (Apache-2.0 §4)

`models/patrol_robot/model.sdf` は TurtleBot3 Waffle Pi の `model.sdf` を元に、次の点を変更しています。

- モデル名を `patrol_robot` に変更
- カメラを高さ 0.8 m・俯角 0.2 rad のマスト上へ移設し、マストの見た目を追加
- LiDAR とカメラの可視化表示を無効化
