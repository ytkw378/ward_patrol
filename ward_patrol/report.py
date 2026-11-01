"""巡回結果を看護記録風の Markdown レポートに整形する."""

STATUS_JA = {
    'OK': '異常なし',
    'FALL': '**転倒を検知**',
    'CHECK': '要確認 (再確認で否定)',
    'UNCHECKED': '未確認',
}
STATE_JA = {'UPRIGHT': '立位', 'IN_BED': '在床', 'FALLEN': '転倒'}


def render_markdown(data):
    lines = [
        '# 夜間巡回レポート',
        '',
        f"- 作成日時: {data['generated']}",
        f"- 巡回時間: {data['duration_sec']:.0f} 秒 (シミュレーション時間)",
        f"- 通報件数: {len(data['alerts'])} 件",
        '',
        '## 病室ごとの結果',
        '',
        '| 病室 | 判定 | 内訳 | 代表画像 |',
        '|---|---|---|---|',
    ]
    for room, st in data['rooms'].items():
        snap = data['snapshots'].get(room)
        img = f'![{room}]({snap})' if snap else '-'
        lines.append(f"| {room} | {STATUS_JA.get(st['status'], st['status'])} "
                     f"| {st['summary']} | {img} |")
    lines += ['', '## 検出された人物', '',
              '| 病室 | 状態 | 位置 (x, y) [m] | 身体の高さ [m] | 観測回数 | 投票 |',
              '|---|---|---|---|---|---|']
    for room, st in data['rooms'].items():
        for p in st.get('persons', []):
            h = '-' if p.get('height') is None else f"{p['height']:.2f}"
            votes = ', '.join(f'{STATE_JA.get(k, k)}:{v}' for k, v in p['votes'].items())
            lines.append(f"| {room} | {STATE_JA.get(p['state'], p['state'])} "
                         f"| ({p['x']:.2f}, {p['y']:.2f}) | {h} | {p['n']} | {votes} |")
    if data['alerts']:
        lines += ['', '## 通報記録', '']
        for a in data['alerts']:
            lines += [
                f"### {a['room']}号室 — 巡回開始から {a['t']:.0f} 秒",
                '',
                f"- 推定位置: ({a['x']:.2f}, {a['y']:.2f}) m",
                f"- 身体の高さ: {a['height']} m / 検出信頼度: {a['conf']:.2f}",
                f"- 通報時刻: {a['wall_time']}",
                '',
                f"![alert]({a['snapshot']})" if a.get('snapshot') else '',
                '',
            ]
    lines += ['', '## 行動ログ', '', '| 経過 [s] | 内容 |', '|---|---|']
    for e in data['events']:
        lines.append(f"| {e['t']:.1f} | {e['text']} |")
    lines.append('')
    return '\n'.join(lines)
