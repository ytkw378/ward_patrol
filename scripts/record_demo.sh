#!/bin/bash
# デモを起動し、noVNC のデスクトップ (:0) 全体を録画する。コンテナ内で実行する。
#   bash /workspace/ros2_ws/src/ward_patrol/scripts/record_demo.sh [scenario] [seed]
# 出力: /workspace/output/video/raw_<scenario>.mp4 (等倍), launch_<scenario>.log
SCENARIO=${1:-A}
SEED=${2:-}
OUT=/workspace/output/video
mkdir -p "$OUT"

source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source "$HOME/airobot_ws/install/setup.bash"
source /workspace/ros2_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=$HOME/.Xauthority
set -u  # (ROS / Gazebo の setup スクリプトは未定義変数を参照するので source の後で有効化)

# スクリーンセーバー (MATE の floaters) が録画に映り込むのを防ぐ
mate-screensaver-command --exit >/dev/null 2>&1
pkill floaters 2>/dev/null
xset s off 2>/dev/null

# 画面左半分に Gazebo、右半分に見守りモニタ (ward_monitor が x=960 に配置)
GUI_INI=$HOME/.gazebo/gui.ini
[ -f "$GUI_INI.orig" ] || cp "$GUI_INI" "$GUI_INI.orig" 2>/dev/null || true
cat > "$GUI_INI" <<EOF
[geometry]
x=0
y=0
width=960
height=1045
EOF

LOG=$OUT/launch_$SCENARIO.log
ros2 launch ward_patrol ward_patrol.launch.py scenario:="$SCENARIO" ${SEED:+layout_seed:=$SEED} > "$LOG" 2>&1 &
LAUNCH_PID=$!

# 巡回ノードが起動したら録画開始 (Gazebo とモニタの描画が落ち着くのを待つ)
until grep -q "patrol_manager-[0-9]*\]: process started" "$LOG"; do sleep 1; done
sleep 2
date +%s.%N > "$OUT/rec_start_$SCENARIO.txt"
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 10 -video_size 1920x1080 -i :0.0 \
  -c:v libx264 -preset ultrafast -crf 24 -pix_fmt yuv420p "$OUT/raw_$SCENARIO.mp4" &
FFMPEG_PID=$!
echo "recording -> $OUT/raw_$SCENARIO.mp4"

until grep -q "巡回完了" "$LOG"; do sleep 2; done
sleep 6
kill -INT $FFMPEG_PID; wait $FFMPEG_PID
kill -INT $LAUNCH_PID 2>/dev/null
bash "$(dirname "$0")/stop_demo.sh"
[ -f "$GUI_INI.orig" ] && cp "$GUI_INI.orig" "$GUI_INI"
grep "巡回完了" "$LOG"
echo "done: $OUT/raw_$SCENARIO.mp4"
