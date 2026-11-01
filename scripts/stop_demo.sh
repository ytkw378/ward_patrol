#!/bin/bash
# デモ関連のプロセスをすべて停止する (パターンを [x] で囲み pkill 自身にマッチさせない)
pkill -INT -f "[r]os2 launch ward_patrol"
sleep 3
for p in "[f]fmpeg -loglevel error -y -f x11grab" "[g]zserver" "[g]zclient" \
         "[w]ard_patrol/lib/ward_patrol" "[r]os2 launch ward_patrol" "[s]pawn_entity.py"; do
  pkill -9 -f "$p"
done
exit 0
