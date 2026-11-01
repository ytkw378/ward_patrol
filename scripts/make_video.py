"""録画 (raw_<scenario>.mp4) から提出用動画を作る。コンテナ内で実行する。

    python3 make_video.py A /workspace/output/patrol_YYYYmmdd_HHMMSS

構成: タイトルカード -> 本編 (2倍速・行動ログを字幕表示) -> 結果カード
"""
import json
import os
import re
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_B = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
W, H = 1920, 1080
SPEED = 2.0
OUT = '/workspace/output/video'

BG = (24, 28, 34)
FG = (236, 236, 236)
SUB = (160, 170, 180)
ACC = (90, 170, 255)
RED = (235, 70, 70)
GREEN = (100, 200, 110)
BLUE = (60, 160, 230)


def font(size, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT, size)


def box(d, xy, text, fill, size=30):
    x0, y0, x1, y1 = xy
    d.rounded_rectangle(xy, 18, outline=fill, width=4, fill=(34, 40, 48))
    d.multiline_text(((x0 + x1) / 2, (y0 + y1) / 2), text, font=font(size, True), fill=FG,
                     anchor='mm', align='center', spacing=8)


def arrow(d, p0, p1, color=SUB):
    d.line([p0, p1], fill=color, width=5)
    x, y = p1
    if p0[1] == p1[1]:
        d.polygon([(x, y), (x - 18, y - 11), (x - 18, y + 11)], fill=color)
    else:
        d.polygon([(x, y), (x - 11, y - 18), (x + 11, y - 18)], fill=color)


def title_card(path, scenario):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((120, 90), 'AI夜間巡回ロボット', font=font(84, True), fill=FG)
    d.text((124, 200), '病棟を見回り、ベッドからの転落・転倒を見つけて知らせる', font=font(40), fill=ACC)
    d.text((124, 262), 'ROS 2 Humble / Gazebo / YOLOv8-Pose / TurtleBot3 Waffle Pi + カメラマスト(0.8 m)',
           font=font(28), fill=SUB)
    # 処理の流れ
    y1, y2 = 400, 560
    box(d, (120, y1, 470, y1 + 110), 'カメラ画像', ACC)
    box(d, (560, y1, 1010, y1 + 110), 'YOLOv8-Pose\n骨格推定 → 姿勢', ACC, 28)
    box(d, (120, y2, 470, y2 + 110), 'LiDAR', ACC)
    box(d, (560, y2, 1010, y2 + 110), '体幹までの距離\n→ 床からの高さ', ACC, 28)
    box(d, (1100, 470, 1450, 600), '姿勢 × 高さ\nで状態判定', (230, 190, 60), 32)
    box(d, (1540, 470, 1840, 600), '巡回\n状態機械', (230, 190, 60), 32)
    arrow(d, (470, y1 + 55), (560, y1 + 55))
    arrow(d, (470, y2 + 55), (560, y2 + 55))
    arrow(d, (1010, y1 + 55), (1100, 520))
    arrow(d, (1010, y2 + 55), (1100, 550))
    arrow(d, (1450, 535), (1540, 535))
    # 判定ルール
    rules = [(GREEN, '立位', '骨格が縦向き'), (BLUE, '在床', '臥位・体幹の高さ ≥ 0.6 m (正常)'),
             (RED, '転倒', '臥位・体幹の高さ < 0.6 m (床の上) → 接近・再確認・通報')]
    for i, (c, name, desc) in enumerate(rules):
        y = 740 + i * 62
        d.ellipse((124, y + 8, 152, y + 36), fill=c)
        d.text((170, y), name, font=font(34, True), fill=FG)
        d.text((270, y + 2), desc, font=font(32), fill=SUB)
    d.text((W - 120, 990), f'シナリオ {scenario}  (配置はロボットに知らされていない)  /  本編は 2倍速再生',
           font=font(28), fill=SUB, anchor='ra')
    img.save(path)


def result_card(path, report):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((120, 90), '巡回結果 (自動生成レポートより)', font=font(64, True), fill=FG)
    d.text((124, 190), f"巡回時間 {report['duration_sec']:.0f} 秒 (シミュレーション時間)   通報 "
           f"{len(report['alerts'])} 件", font=font(34), fill=SUB)
    colors = {'OK': GREEN, 'FALL': RED, 'CHECK': (230, 190, 60), 'UNCHECKED': SUB}
    label = {'OK': '異常なし', 'FALL': '転倒を検知・通報', 'CHECK': '要確認', 'UNCHECKED': '未確認'}
    for i, (room, st) in enumerate(report['rooms'].items()):
        y = 290 + i * 130
        c = colors.get(st['status'], SUB)
        d.rounded_rectangle((120, y, 1100, y + 110), 18, fill=(34, 40, 48))
        d.rectangle((120, y, 136, y + 110), fill=c)
        d.text((170, y + 22), f'{room}号室', font=font(48, True), fill=FG)
        d.text((420, y + 14), label.get(st['status'], ''), font=font(36, True), fill=c)
        d.text((420, y + 62), st['summary'], font=font(28), fill=SUB)
    snap = None
    if report['alerts'] and report['alerts'][0].get('snapshot'):
        snap = os.path.join(report['_dir'], report['alerts'][0]['snapshot'])
    if snap and os.path.exists(snap):
        im = Image.open(snap).resize((640, 480))
        img.paste(im, (1180, 290))
        a = report['alerts'][0]
        d.text((1180, 790), f"通報時の画像 ({a['room']}号室)", font=font(30, True), fill=RED)
        d.text((1180, 836), f"推定位置 ({a['x']:.1f}, {a['y']:.1f}) m / 体幹の高さ {a['height']:.2f} m",
               font=font(26), fill=SUB)
    d.text((124, 720), '・病室・人物の配置はロボットに知らせず、AI の検出結果だけで行動を決定',
           font=font(30), fill=SUB)
    d.text((124, 770), '・遠距離で疑い → 接近して近距離で再確認してから通報 (誤報の抑制)',
           font=font(30), fill=SUB)
    d.text((124, 820), '・ベッド上の臥床は「在床」として正常扱い (姿勢だけでは区別できない)',
           font=font(30), fill=SUB)
    d.text((124, 870), '・判定根拠 (推論ログ・画像) を Markdown レポートとして保存',
           font=font(30), fill=SUB)
    img.save(path)


def captions(log_path, rec_start):
    """launch ログの巡回ノードの出力から (動画上の秒, テキスト) の字幕を作る."""
    pat = re.compile(r'\[patrol_manager\]: \[\s*[\d.]+s\] (.+)$')
    stamp = re.compile(r'\[INFO\] \[([\d.]+)\]')
    out = []
    for line in open(log_path, encoding='utf-8'):
        m, s = pat.search(line), stamp.search(line)
        if m and s:
            text = m.group(1)
            if text.startswith('巡回完了'):
                text = '巡回完了 → レポートを自動生成'
            out.append(((float(s.group(1)) - rec_start) / SPEED, text))
    return out


def esc(text):
    return text.replace('\\', '\\\\').replace(':', '\\:').replace("'", "’").replace('%', '\\%')


def main():
    scenario = sys.argv[1] if len(sys.argv) > 1 else 'A'
    report_dir = sys.argv[2]
    with open(os.path.join(report_dir, 'report.json')) as f:
        report = json.load(f)
    report['_dir'] = report_dir
    raw = os.path.join(OUT, f'raw_{scenario}.mp4')
    rec_start = float(open(os.path.join(OUT, f'rec_start_{scenario}.txt')).read())
    caps = captions(os.path.join(OUT, f'launch_{scenario}.log'), rec_start)
    # 冒頭 (起動・モデル読み込み) を切り、最初のイベントの 4 秒前から使う
    skip = max(0.0, caps[0][0] * SPEED - 4.0) if caps else 0.0
    caps = [(t - skip / SPEED, text) for t, text in caps]

    title_card(os.path.join(OUT, 'card_title.png'), scenario)
    result_card(os.path.join(OUT, 'card_result.png'), report)

    # 本編: Gazebo の 3D ビューとモニタを切り出して横に並べ、2倍速 + 行動ログの字幕
    #   Gazebo の描画領域 (サイドパネル・ツールバー・追従モード表示を除く) と
    #   モニタウィンドウの中身 (タイトルバーを除く) の画面上の位置
    gz_crop = 'crop=668:825:272:150,scale=872:1080'
    mon_crop = 'crop=958:988:962:62,scale=1048:1080'
    fb = FONT_B
    graph = [f'[0:v]setpts=PTS/{SPEED},split[g][m]', f'[g]{gz_crop}[gz]', f'[m]{mon_crop}[mon]',
             '[gz][mon]hstack=inputs=2[s0]']
    draws = [f"drawtext=fontfile={fb}:text='▶▶ 2倍速再生':x=20:y=20:fontsize=32:"
             "fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=10",
             f"drawtext=fontfile={fb}:text='Gazebo (ロボット後方から追従)':x=20:y=76:fontsize=24:"
             "fontcolor=white:box=1:boxcolor=black@0.4:boxborderw=8"]
    for i, (t0, text) in enumerate(caps):
        t1 = caps[i + 1][0] if i + 1 < len(caps) else t0 + 6
        t1 = min(t1, t0 + 8)
        color = 'ff5050' if ('緊急' in text or '転倒の疑い' in text) else 'ffffff'
        draws.append(f"drawtext=fontfile={fb}:text='{esc(text)}':x=24:y=h-120:fontsize=34:"
                     f"fontcolor=0x{color}:box=1:boxcolor=black@0.65:boxborderw=14:"
                     f"enable='between(t,{max(t0, 0):.2f},{t1:.2f})'")
    graph.append('[s0]' + ','.join(draws) + '[v]')
    main_mp4 = os.path.join(OUT, f'main_{scenario}.mp4')
    subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-ss', f'{skip:.2f}', '-i', raw,
                    '-filter_complex', ';'.join(graph), '-map', '[v]',
                    '-an', '-r', '30', '-c:v', 'libx264', '-preset', 'medium', '-crf', '22',
                    '-pix_fmt', 'yuv420p', main_mp4], check=True)

    final = os.path.join(OUT, f'ward_patrol_demo_{scenario}.mp4')
    subprocess.run([
        'ffmpeg', '-loglevel', 'error', '-y',
        '-loop', '1', '-t', '8', '-i', os.path.join(OUT, 'card_title.png'),
        '-i', main_mp4,
        '-loop', '1', '-t', '7', '-i', os.path.join(OUT, 'card_result.png'),
        '-filter_complex',
        '[0:v]scale=1920:1080,fps=30,format=yuv420p,setsar=1[a];'
        '[1:v]scale=1920:1080,fps=30,format=yuv420p,setsar=1[b];'
        '[2:v]scale=1920:1080,fps=30,format=yuv420p,setsar=1[c];'
        '[a][b][c]concat=n=3:v=1:a=0[v]',
        '-map', '[v]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '22',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', final], check=True)
    dur = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                          '-of', 'csv=p=0', final], capture_output=True, text=True).stdout
    print(f'{final}  ({float(dur):.1f} s)')


if __name__ == '__main__':
    main()
