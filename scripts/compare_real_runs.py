#!/usr/bin/env python3
"""Đối chiếu Nav-only, PI và SAC80k xe thật; không mở ROS/điều khiển motor."""
import argparse
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import statistics
import zipfile


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    x = (len(values) - 1) * q
    lo = int(x)
    return values[lo] + (values[math.ceil(x)] - values[lo]) * (x - lo)


def weighted_rms(rows, intervals, extract):
    pairs = []
    for i, dt in intervals:
        try:
            values = extract(rows[i])
            if values and all(number(v) for v in values):
                pairs.append((statistics.mean(v * v for v in values), dt))
        except (KeyError, TypeError, IndexError, ValueError):
            pass
    duration = sum(dt for _, dt in pairs)
    return math.sqrt(sum(v * dt for v, dt in pairs) / duration) if duration else None


def load_measurements(directory, manifest):
    """Cùng cửa sổ goal và logger thụ động cho cả ba controller."""
    summary = json.loads((directory/'measurements/summary.json').read_text())
    if summary.get('group') != manifest['group'] or summary.get('format_version') != 1:
        raise ValueError('Wrong measurement schema/group: '+str(directory))
    if summary.get('closed') is not True:
        raise ValueError('Measurement logger not closed cleanly; preserve and inspect session: '+str(directory))
    goals = summary['goals']
    invalid = None if len(goals) == 1 else f'Expected one goal per session, found {len(goals)}'
    key, goal = next(iter(goals.items())) if len(goals) == 1 else (None, {})
    start, end = goal.get('start'), goal.get('terminal')
    rejected = 0
    def read(kind):
        nonlocal rejected
        rows = []
        for line in (directory/'measurements'/f'{kind}.jsonl').read_text().splitlines():
            try:
                row = json.loads(line)
                t = row.get('ros_stamp')
                if (invalid or row.get('goal_id') != key or not number(t) or
                    not number(start) or t < start or (end is not None and t > end) or
                    (rows and t <= rows[-1]['ros_stamp'])):
                    rejected += 1
                    continue
                rows.append(row)
            except (ValueError, TypeError):
                rejected += 1
        return rows
    rows, imu = read('pose'), read('imu')
    def intervals(data):
        return [(i, data[i+1]['ros_stamp']-data[i]['ros_stamp']) for i in range(len(data)-1)
                if 0 < data[i+1]['ros_stamp']-data[i]['ros_stamp'] <= .25]
    dt, imu_dt = intervals(rows), intervals(imu)
    errors = [abs(r['cross_track']) for r in rows if number(r.get('cross_track'))]
    torques = [abs(v) for r in rows for v in r.get('total', []) if number(v)]
    def wheel_error(r):
        a, b = r['wheel_target'], r['wheel_velocity']
        if len(a) != 2 or len(b) != 2:
            raise ValueError('Wheel vectors must have two values')
        return [x-y for x,y in zip(a,b)]
    outcome = json.loads((directory/'outcome.json').read_text())
    nav = {4:'SUCCEEDED', 5:'CANCELED', 6:'ABORTED'}.get(goal.get('status'), 'UNKNOWN')
    duration = end-start if nav == 'SUCCEEDED' and number(start) and number(end) and end >= start else None
    metrics = dict(
        armed_valid_samples=None, valid_pose_samples=len(rows), imu_source_samples=len(imu),
        rejected_rows=rejected, logger_rejected_samples=sum(summary.get('rejected', {}).values()),
        observed_span_s=rows[-1]['ros_stamp']-rows[0]['ros_stamp'] if rows else None,
        weighted_coverage_s=sum(d for _,d in dt), imu_weighted_coverage_s=sum(d for _,d in imu_dt),
        amcl_cross_track_rmse_m=weighted_rms(rows, dt, lambda r:[r['cross_track']]),
        amcl_cross_track_p95_m=percentile(errors,.95), amcl_cross_track_max_m=max(errors) if errors else None,
        amcl_heading_rmse_rad=weighted_rms(rows, dt, lambda r:[r['heading_error']]),
        wheel_tracking_rmse_rad_s=weighted_rms(rows,dt,wheel_error),
        imu_lateral_accel_rms_m_s2=weighted_rms(imu,imu_dt,lambda r:[r['acceleration'][1]]),
        imu_vertical_accel_rms_m_s2=weighted_rms(imu,imu_dt,lambda r:[r['acceleration'][2]]),
        command_torque_rms_nm=weighted_rms(rows,dt,lambda r:r['total']),
        command_torque_peak_nm=max(torques) if torques else None,
        nav2_outcome_from_status=nav, invalid_session_reason=invalid,
        logger_closed=summary.get('closed',False),
        nav2_outcome_operator_annotation=outcome.get('nav2_outcome'),
        amcl_task_pass_operator_annotation=outcome.get('amcl_task_pass'),
        operator_stop_reason=outcome.get('operator_stop_reason'), completion_time_s=duration,
        terminal_goal_xy_m=None, terminal_goal_yaw_rad=None, actuator_rate_saturation_pct=None)
    metrics['gaps_over_0.25s'] = max(0,len(rows)-1-len(dt))
    return dict(directory=str(directory),manifest=manifest,metrics=metrics,rows=rows,measurement=summary)


def load_run(directory, expected):
    directory = Path(directory).resolve()
    manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest['group'] != expected:
        raise ValueError(f'{directory}: expected {expected}, got {manifest["group"]}')
    for entry in manifest['files']:
        data = (directory / entry['snapshot']).read_bytes()
        if hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise ValueError('Snapshot hash mismatch: ' + entry['snapshot'])
    if expected == 'sac80k':
        model = next(e for e in manifest['files'] if Path(e['source']).name == 'model.zip')
        contract = next(e for e in manifest['files'] if Path(e['source']).name == 'contract.json')
        contract = json.loads((directory / contract['snapshot']).read_text())
        if model['sha256'] != contract['sha256']['model.zip']:
            raise ValueError('Checkpoint differs from imported contract')
        with zipfile.ZipFile(directory / model['snapshot']) as archive:
            steps = json.loads(archive.read('data'))['num_timesteps']
        if steps != 80000 or contract['timesteps'] != 80000:
            raise ValueError(f'Expected checkpoint 80000, got {steps}')
    if manifest.get('measurement_source') == 'passive_v1':
        return load_measurements(directory, manifest)
    if expected == 'nav_only':
        raise ValueError('Nav-only needs passive_v1 measurement logs')
    logs = list((directory / 'controller').glob('real_*.jsonl'))
    if len(logs) != 1:
        raise ValueError(f'{directory}: need exactly one controller JSONL')
    rows, rejected = [], 0
    mode = 'pi' if expected == 'pi' else 'pi_sac'
    for line in logs[0].read_text().splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            rejected += 1
            continue
        if row.get('mode') != mode:
            raise ValueError('Controller mode does not match session manifest')
        if row.get('armed') is not True:
            continue
        t = row.get('ros_stamp')
        if not number(t) or (rows and t <= rows[-1]['ros_stamp']):
            rejected += 1
            continue
        rows.append(row)
    intervals, gaps = [], 0
    for i in range(len(rows) - 1):
        dt = rows[i + 1]['ros_stamp'] - rows[i]['ros_stamp']
        if dt <= .25:
            intervals.append((i, dt))
        else:
            gaps += 1
    errors = [abs(r['cross_track']) for r in rows if number(r.get('cross_track'))]
    torques = [abs(v) for r in rows for v in r.get('total', []) if number(v)]
    def wheel_error(r):
        a, b = r['wheel_target'], r['wheel_velocity']
        if len(a) != 2 or len(b) != 2:
            raise ValueError('Wheel vectors must have two values')
        return [x - y for x, y in zip(a, b)]
    outcome_file = directory / 'outcome.json'
    outcome = json.loads(outcome_file.read_text()) if outcome_file.exists() else {}
    metrics = {
        'armed_valid_samples': len(rows),
        'rejected_rows': rejected,
        'gaps_over_0.25s': gaps,
        'observed_span_s': rows[-1]['ros_stamp'] - rows[0]['ros_stamp'] if rows else None,
        'weighted_coverage_s': sum(dt for _, dt in intervals),
        'amcl_cross_track_rmse_m': weighted_rms(rows, intervals, lambda r: [r['cross_track']]),
        'amcl_cross_track_p95_m': percentile(errors, .95),
        'amcl_cross_track_max_m': max(errors) if errors else None,
        'amcl_heading_rmse_rad': weighted_rms(rows, intervals, lambda r: [r['heading_error']]),
        'wheel_tracking_rmse_rad_s': weighted_rms(rows, intervals, wheel_error),
        'imu_lateral_accel_rms_m_s2': weighted_rms(rows, intervals, lambda r: [r['acceleration'][1]]),
        'imu_vertical_accel_rms_m_s2': weighted_rms(rows, intervals, lambda r: [r['acceleration'][2]]),
        'command_torque_rms_nm': weighted_rms(rows, intervals, lambda r: r['total']),
        'command_torque_peak_nm': max(torques) if torques else None,
        'nav2_outcome_operator_annotation': outcome.get('nav2_outcome'),
        'amcl_task_pass_operator_annotation': outcome.get('amcl_task_pass'),
        'operator_stop_reason': outcome.get('operator_stop_reason'),
        'completion_time_s': None,
        'terminal_goal_xy_m': None,
        'terminal_goal_yaw_rad': None,
        'actuator_rate_saturation_pct': None,
    }
    return dict(directory=str(directory), manifest=manifest, metrics=metrics, rows=rows)


def chart(run):
    samples = [r for r in run['rows'] if number(r.get('cross_track'))]
    if not samples:
        return '<p>N/A: không có mẫu armed hợp lệ.</p>'
    t0 = samples[0]['ros_stamp']
    span = max(samples[-1]['ros_stamp'] - t0, .001)
    scale = max(.2, max(abs(r['cross_track']) for r in samples))
    parts = ['<svg viewBox="0 0 700 230" role="img" aria-label="Lỗi ngang theo thời gian">']
    for value in (-.2, 0, .2):
        y = 110 - value / scale * 85
        parts.append(f'<path d="M40 {y}H680" stroke="#aaa" stroke-dasharray="4 4"/>')
    # Không nối qua khoảng mất dữ liệu.
    segments, current = [], []
    previous = None
    for row in samples:
        t = row['ros_stamp']
        if previous is not None and t - previous > .25:
            segments.append(current)
            current = []
        current.append(f'{40+(t-t0)/span*640:.2f},{110-row["cross_track"]/scale*85:.2f}')
        previous = t
    segments.append(current)
    for segment in segments:
        parts.append(f'<polyline points="{" ".join(segment)}" fill="none" stroke="#1768ac"/>')
    parts.append(f'<text x="40" y="218">0 → {span:.2f} s; e_y ±{scale:.3f} m; nét đứt: ±0,20 m và zero</text></svg>')
    return ''.join(parts)


def build_report(pi_dir, sac_dir, output_dir):
    runs = [load_run(pi_dir, 'pi'), load_run(sac_dir, 'sac80k')]
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    def show(value):
        return 'N/A' if value is None else (f'{value:.6g}' if isinstance(value, float) else str(value))
    with (output / 'summary.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'PI', 'PI+SAC80k'])
        for key in runs[0]['metrics']:
            writer.writerow([key, *(show(r['metrics'][key]) for r in runs)])
    warnings = []
    for name in ('real.yaml', 'robot.yaml', 'bridge.yaml', 'nav2.yaml'):
        hashes = [{e['sha256'] for e in r['manifest']['files'] if Path(e['source']).name == name}
                  for r in runs]
        if hashes[0] != hashes[1]:
            warnings.append('Khác cấu hình ' + name)
    map_hashes = [{e['sha256'] for e in r['manifest']['files']
                   if '/maps/' in e['source']} for r in runs]
    if map_hashes[0] != map_hashes[1]:
        warnings.append('Map/ảnh map khác nhau')
    warnings.append('Chưa tự xác minh cùng điểm đầu/goal hoặc cùng đường tham chiếu; kiểm tra rosbag.')
    warnings.append('Thời gian hoàn thành và terminal goal error: N/A; JSONL không ghi terminal pose/outcome có timestamp.')
    warnings.append('Chỉ phân tích các mẫu armed có observation hợp lệ; phần thiếu và lỗi phải đối chiếu status/rosbag, không loại lượt thất bại.')
    body = ['<!doctype html><meta charset="utf-8"><title>PI vs SAC80k — xe thật</title>',
            '<style>body{font:16px sans-serif;max-width:1100px;margin:30px auto;padding:16px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:9px}svg{width:100%}code{overflow-wrap:anywhere}</style>',
            '<h1>PI và PI+SAC80k — hai phiên xe thật</h1>',
            '<p>Mỗi controller một phiên; chưa đủ kết luận thống kê. Pose: AMCL+odom TF, IMU đã trừ trọng lực, bánh: encoder; mô-men là lệnh, không phải đo thực.</p>',
            '<p>RMSE cân theo dt, giữ giá trị bên trái mỗi khoảng 0 &lt; dt ≤ 0,25 s; không nối qua gap. P95 là phân vị mẫu trong phiên. Torque chỉ là snapshot nhịp actor, không đại diện mọi lệnh publish.</p>',
            '<ul>' + ''.join('<li>' + html.escape(w) + '</li>' for w in warnings) + '</ul>',
            '<p>Outcome nếu được điền là nhãn của người vận hành, chưa tự xác minh. Không suy ra pass từ RMSE hoặc Nav2 SUCCEEDED.</p>',
            '<table><tr><th>Chỉ số</th><th>PI</th><th>PI+SAC80k</th></tr>']
    for key in runs[0]['metrics']:
        body.append('<tr><td>' + html.escape(key) + '</td>' + ''.join(
            '<td>' + html.escape(show(r['metrics'][key])) + '</td>' for r in runs) + '</tr>')
    body.append('</table>')
    for label, run in zip(('PI', 'PI+SAC80k'), runs):
        body.append(f'<h2>{label}</h2><code>{html.escape(run["directory"])}</code>' + chart(run))
    (output / 'report.html').write_text('\n'.join(body))
    return output / 'report.html'


def build_group_report(pi_dirs, sac_dirs, output_dir, nav_dirs=None):
    """Gộp chỉ số theo phiên, không trộn mọi mẫu thành một lượt dài."""
    paths = [Path(p).resolve() for p in [*(nav_dirs or []), *pi_dirs, *sac_dirs]]
    if len(set(paths)) != len(paths):
        raise ValueError('Duplicate session directory; one session cannot count twice')
    groups = [([load_run(p, 'pi') for p in pi_dirs], 'PI'),
              ([load_run(p, 'sac80k') for p in sac_dirs], 'PI+SAC80k')]
    if nav_dirs is not None:
        groups.insert(0, ([load_run(p,'nav_only') for p in nav_dirs], 'Nav-only'))
    if not all(runs for runs, _ in groups):
        raise ValueError('Need at least one session per group')
    sources = {r['manifest'].get('measurement_source','legacy_actor') for group,_ in groups for r in group}
    if len(sources) != 1 or (nav_dirs is not None and sources != {'passive_v1'}):
        raise ValueError('Do not mix actor snapshots and passive measurement protocols; record all groups with the same logger')
    passive = sources == {'passive_v1'}
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    keys = list(dict.fromkeys(k for group,_ in groups for r in group for k in r['metrics']))
    aggregate = {}
    with (output / 'episodes.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['group', 'session', *keys])
        for runs, label in groups:
            for run in runs:
                writer.writerow([label, run['directory'], *[
                    'N/A' if run['metrics'].get(k) is None else run['metrics'][k] for k in keys]])
    with (output / 'summary.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['group', 'metric', 'valid_episodes', 'total_episodes', 'mean', 'sample_std'])
        for runs, label in groups:
            aggregate[label] = {}
            for key in keys:
                values = [r['metrics'][key] for r in runs if number(r['metrics'].get(key))]
                mean = statistics.mean(values) if values else None
                std = statistics.stdev(values) if len(values) > 1 else None
                aggregate[label][key] = dict(n=len(values), mean=mean, std=std)
                writer.writerow([label, key, len(values), len(runs),
                                 'N/A' if mean is None else mean, 'N/A' if std is None else std])
    def cell(stat):
        if stat['mean'] is None:
            return 'N/A'
        std = f'{stat["std"]:.5g}' if stat['std'] is not None else 'N/A'
        return f'{stat["mean"]:.5g} ± {std} (n={stat["n"]})'
    title = 'Nav-only, PI và PI+SAC80k' if nav_dirs is not None else 'PI và PI+SAC80k'
    body = ['<!doctype html><meta charset="utf-8"><title>'+title+' — xe thật</title>',
            '<style>body{font:16px sans-serif;max-width:1100px;margin:30px auto;padding:16px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:8px}svg{width:100%}</style>',
            '<h1>'+title+' — xe thật</h1>',
            '<p>Trung bình ± độ lệch chuẩn mẫu giữa các lượt. Không phải độ lệch chuẩn của tất cả mẫu cảm biến. Không tự loại lượt thất bại.</p>',
            '<p>Pose theo AMCL + odometry TF, không phải vị trí thực tuyệt đối. '+
            ('Cả ba nhóm dùng cùng logger thụ động: pose 10 Hz, IMU ở nhịp nguồn (~50 Hz), gia tốc đã trừ trọng lực. Chỉ lấy cửa sổ goal, loại thời gian reset/bê xe. '
             if passive else 'Rung dùng JSONL actor khoảng 10 Hz; có thể bỏ sót rung tần số cao. ')+
            'Dữ liệu gốc giữ trong rosbag. Mô-men là snapshot lệnh controller; Nav-only không có nguồn lệnh mô-men nên N/A.</p>']
    for runs, label in groups:
        annotations = [r['metrics']['amcl_task_pass_operator_annotation'] for r in runs]
        known = sum(type(x) is bool for x in annotations)
        passed = sum(x is True for x in annotations)
        failed = sum(x is False for x in annotations)
        body.append(f'<p>{label}: {len(runs)} phiên; AMCL pass do người vận hành gán nhãn: '
                    f'{passed} đạt, {failed} không đạt, {len(runs)-known} chưa có nhãn. '
                    'Chưa tự xác minh outcome từ rosbag.</p>')
        if passive:
            outcomes = [r['metrics']['nav2_outcome_from_status'] for r in runs]
            success = outcomes.count('SUCCEEDED')
            failure = sum(x in ('ABORTED','CANCELED') for x in outcomes)
            invalid = sum(bool(r['metrics']['invalid_session_reason']) for r in runs)
            body.append(f'<p>{label}: Nav2 SUCCEEDED {success}/{len(runs)}; ABORTED/CANCELED {failure}; '
                        f'chưa có terminal {outcomes.count("UNKNOWN")}; phiên nhiều/thiếu goal {invalid}. '
                        'Nav2 outcome không đồng nghĩa AMCL task pass.</p>')
    # Kiểm tra tính đồng nhất cấu hình theo toàn bộ phiên, không chỉ từng cặp.
    runs = [run for group, _ in groups for run in group]
    problems = []
    for name in ('real.yaml', 'robot.yaml', 'bridge.yaml', 'nav2.yaml'):
        signatures = [frozenset(e['sha256'] for e in r['manifest']['files']
                               if Path(e['source']).name == name) for r in runs]
        if any(not s for s in signatures) or len(set(signatures)) != 1:
            problems.append('Thiếu/khác snapshot ' + name)
    if passive:
        for name in ('core.py','node.py','features.py','measure_real_run.py'):
            signatures = [frozenset(e['sha256'] for e in r['manifest']['files']
                                   if Path(e['source']).name == name) for r in runs]
            if any(not s for s in signatures) or len(set(signatures)) != 1:
                problems.append('Thiếu/khác code snapshot ' + name)
    maps = [frozenset(e['sha256'] for e in r['manifest']['files'] if '/maps/' in e['source']) for r in runs]
    if any(not s for s in maps) or len(set(maps)) != 1:
        problems.append('Thiếu/khác snapshot map')
    problems.append('Chưa tự xác minh cùng start/goal/path; phải đối chiếu rosbag. Không kết luận SAC tốt hơn từ nhóm ít hoàn thành hơn hoặc thiếu dữ liệu.')
    if passive:
        hashes = [frozenset(v['sha256'] for v in r['measurement']['references'].values()) for r in runs]
        if any(not h for h in hashes) or len(set(hashes)) != 1:
            problems.append('Đường tham chiếu đầu tiên khác/thiếu giữa các lượt; RMSE tính theo đường riêng từng lượt, không tuyên bố cùng quỹ đạo.')
        problems.append('Mỗi lượt chỉ một goal. Thời gian hoàn thành tính từ timestamp goal đến lần đầu nhận status SUCCEEDED, gồm độ trễ status. Lỗi đích chưa đủ nguồn thì N/A. Wheel tracking dùng tham chiếu động học trước slew của controller.')
    body.append('<ul>' + ''.join('<li>' + html.escape(p) + '</li>' for p in problems) + '</ul>')
    body.append('<table><tr><th>Chỉ số</th>'+''.join('<th>'+label+' Mean ± Std</th>' for _,label in groups)+'</tr>')
    for key in keys:
        if not any(aggregate[label][key]['n'] for _, label in groups):
            continue
        body.append('<tr><td>' + html.escape(key) + '</td>' + ''.join(
            '<td>' + cell(aggregate[label][key]) + '</td>' for _, label in groups) + '</tr>')
    body.append('</table><h2>Thay đổi trung bình so với PI</h2>')
    for key in ('amcl_cross_track_rmse_m', 'imu_lateral_accel_rms_m_s2', 'imu_vertical_accel_rms_m_s2'):
        pi, sac = aggregate['PI'][key], aggregate['PI+SAC80k'][key]
        sizes = {label:len(group) for group,label in groups}
        if pi['n'] == sizes['PI'] and sac['n'] == sizes['PI+SAC80k'] and pi['mean'] > 1e-12:
            reduction = 100 * (pi['mean'] - sac['mean']) / pi['mean']
            value = f'{reduction:.2f}% (dương: giảm; âm: tăng)'
        else:
            value = 'N/A: thiếu lượt hợp lệ hoặc trung bình PI gần zero'
        body.append('<p>' + html.escape(key + ': ' + value) + '</p>')
    body.append('<p>Phần trăm là mô tả số liệu, chưa là bằng chứng có ý nghĩa thống kê hoặc cùng điều kiện. RMSE cân dt ≤0,25 s, bỏ khoảng mất dữ liệu; P95 tính theo mẫu trong từng phiên. Thời gian hoàn thành/terminal error chưa có nguồn đủ thì N/A trong episodes.csv.</p>')
    for group, label in groups:
        for i, run in enumerate(group, 1):
            body.append(f'<h2>{label} — lượt {i}</h2><p>{html.escape(run["directory"])}</p>' + chart(run))
    (output / 'report.html').write_text('\n'.join(body))
    return output / 'report.html'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pi-dir', required=True, nargs='+', help='Một hoặc nhiều thư mục phiên PI')
    parser.add_argument('--sac-dir', required=True, nargs='+', help='Một hoặc nhiều thư mục phiên SAC80k')
    parser.add_argument('--nav-dir', nargs='+', help='Các phiên Nav-only ghi bằng cùng logger thụ động')
    parser.add_argument('--expected-episodes',type=int,help='Kiểm tra đủ đúng số phiên mỗi nhóm')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    if args.expected_episodes is not None and (args.expected_episodes < 1 or any(
            len(group) != args.expected_episodes for group in [args.pi_dir,args.sac_dir]+([args.nav_dir] if args.nav_dir is not None else []))):
        parser.error('Số phiên mỗi nhóm phải bằng --expected-episodes')
    passive_input = any(json.loads((Path(p)/'manifest.json').read_text()).get('measurement_source') == 'passive_v1'
                        for p in [*args.pi_dir,*args.sac_dir])
    if args.nav_dir is None and len(args.pi_dir) == len(args.sac_dir) == 1 and not passive_input:
        print(build_report(args.pi_dir[0], args.sac_dir[0], args.output_dir))
    else:
        print(build_group_report(args.pi_dir, args.sac_dir, args.output_dir,args.nav_dir))
