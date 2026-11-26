"""Episode Evaluator — DÙNG CHUNG Fast 2D và Gazebo (Master Plan v3 §6.2).

Đầu vào: một "episode record" chuẩn (dict) do runner tạo:
  terminal_state  trạng thái cuối của goal_path_bridge ('none' nếu không có)
  t_terminal      s từ lúc gửi goal (âm nếu không có)
  trace           ndarray N×≥6 [t, x, y, yaw, v, w] (ground truth, t từ lúc gửi goal)
  min_clearance   m, khoảng cách GT thân robot -> vật cản nhỏ nhất (< 0 = chạm)
  infra_error     '' hoặc mô tả lỗi hạ tầng (node chết, sensor mất, RTF thấp, lỗi mô phỏng)
  feasibility     'feasible' | 'unsolvable' (nhãn của scenario, từ generator — KHÔNG từ kết quả chạy)
  goal, timeout, sim ('fast2d' | 'gazebo')
  localization    'perfect' (Fast 2D, Gazebo GT) | 'amcl' (Gazebo Full-system); mặc định suy từ sim
Nhãn (ưu tiên từ trên xuống): SYSTEM_ERROR (lỗi hạ tầng), COLLISION, SUCCESS, LOC_LIMITED, SAFE_STOP, STUCK, TIMEOUT.
V4 (2026-11-26): MỘT dung sai đích GT `success_xy` (0.15 m = goal checker) cho mọi simulator. SUCCESS = thành công
VẬT LÝ (GT trong dung sai). LOC_LIMITED (chỉ khi định vị AMCL) = bridge SUCCEEDED, GT ngoài dung sai nhưng lệch giải
thích được bằng sai số AMCL: tính vào Planner Success, KHÔNG tính vào Physical Success (SR).
SAFE_STOP chỉ có ở scenario vô nghiệm; ở scenario khả thi, FAILED/ABORTED/không tiến = STUCK (ghi sub_reason),
nên Safe Stop không bao giờ bị tính nhầm là Stuck và ngược lại.
"""

import math

import numpy as np

LABELS = ['SUCCESS', 'LOC_LIMITED', 'COLLISION', 'STUCK', 'SAFE_STOP', 'TIMEOUT', 'SYSTEM_ERROR']


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stationary_hold(trace, t_from, v_max, w_max, hold):
    """True nếu từ t_from có một khoảng >= hold mà |v| <= v_max và |w| <= w_max liên tục (đến hết trace)."""
    if trace is None or len(trace) == 0:
        return False
    tr = trace[trace[:, 0] >= t_from]
    if len(tr) == 0:
        return False
    ok = (np.abs(tr[:, 4]) <= v_max) & (np.abs(tr[:, 5]) <= w_max)
    # đoạn cuối liên tục thỏa
    if not ok[-1]:
        return False
    k = len(ok) - 1
    while k > 0 and ok[k - 1]:
        k -= 1
    return tr[-1, 0] - tr[k, 0] >= hold - 1e-6


def no_progress_window(trace, window, radius):
    """True nếu tồn tại cửa sổ dài window mà robot dịch < radius so với mọi điểm trong cửa sổ (kẹt)."""
    if trace is None or len(trace) < 2 or trace[-1, 0] - trace[0, 0] < window:
        return False
    t, x, y = trace[:, 0], trace[:, 1], trace[:, 2]
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > window:
            j += 1
        if t[i] - t[j] >= window - 1e-6:
            seg = slice(j, i + 1)
            if np.max(np.hypot(x[seg] - x[i], y[seg] - y[i])) < radius:
                return True
    return False


def evaluate(rec, ev):
    """Trả dict {label, sub_reason, metrics...}. ev = cfg['eval']."""
    out = {'label': None, 'sub_reason': ''}
    trace = rec.get('trace')
    trace = np.asarray(trace) if trace is not None and len(trace) else np.zeros((0, 6))
    st = rec.get('terminal_state', 'none') or 'none'
    feasible = rec['feasibility'] == 'feasible'
    gx, gy, gyaw = rec['goal']
    # sai số đích theo GT tại lúc bridge báo trạng thái cuối (goal checker), và tại lúc dừng hẳn (cuối trace).
    # Pilot2: vài C1 "SUCCEEDED" nhưng sau ramp giảm tốc 0.5 s robot trôi ra > 0.15 m -> chấm tại t_terminal,
    # ghi riêng sai số lúc dừng hẳn.
    # V4: NỘI SUY x, y tại t_terminal (Gazebo: trạng thái bridge đến lệch pha với trace 10 Hz; Fast 2D: trùng tick).
    k = len(trace) - 1
    t_term = rec.get('t_terminal', -1.0)
    if len(trace) and t_term >= 0:
        k = int(np.argmin(np.abs(trace[:, 0] - t_term)))
    if len(trace) > 1 and t_term >= 0 and trace[0, 0] <= t_term <= trace[-1, 0]:
        fx = float(np.interp(t_term, trace[:, 0], trace[:, 1]))
        fy = float(np.interp(t_term, trace[:, 0], trace[:, 2]))
        fyaw = trace[k, 3]
    else:
        fx, fy, fyaw = (trace[k, 1], trace[k, 2], trace[k, 3]) if len(trace) else (math.nan,) * 3
    out['gt_xy_error'] = float(math.hypot(fx - gx, fy - gy)) if len(trace) else math.inf
    out['gt_yaw_error'] = float(abs(_wrap(fyaw - gyaw))) if len(trace) else math.inf
    out['rest_xy_error'] = float(math.hypot(trace[-1, 1] - gx, trace[-1, 2] - gy)) if len(trace) else math.inf
    # dung sai đích GT = goal checker + slack đồng bộ thời gian (TF/GT 20–50 Hz lệch pha); như nhau mọi simulator
    xy_tol = ev['success_xy'] + ev.get('success_xy_slack', 0.0)
    loc = rec.get('localization') or ('amcl' if rec.get('sim') == 'gazebo' else 'perfect')
    out['localization'] = loc

    if rec.get('infra_error'):
        out['label'], out['sub_reason'] = 'SYSTEM_ERROR', rec['infra_error']
    elif rec.get('min_clearance', math.inf) < -ev['collision_tolerance']:
        out['label'] = 'COLLISION'
    elif st == 'SUCCEEDED':
        if not feasible:
            out['label'], out['sub_reason'] = 'SYSTEM_ERROR', 'SUCCEEDED on an unsolvable scenario (label error)'
        elif out['gt_xy_error'] <= xy_tol and out['gt_yaw_error'] <= ev['success_yaw'] + 0.02:
            out['label'] = 'SUCCESS'
        elif loc == 'amcl' and rec.get('loc_error_at_terminal') is not None and \
                out['gt_xy_error'] <= xy_tol + rec['loc_error_at_terminal'] + ev['loc_limited_slack']:
            # Full-system: goal checker đạt theo AMCL; lệch GT giải thích được bằng sai số định vị -> Planner
            # Success nhưng KHÔNG phải Physical Success (V4).
            out['label'], out['sub_reason'] = 'LOC_LIMITED', f"loc error {rec['loc_error_at_terminal']:.2f} m"
        else:
            out['label'], out['sub_reason'] = 'STUCK', 'SUCCEEDED_BUT_OFF_GOAL'
    elif not feasible:
        t0 = rec.get('t_terminal', -1.0)
        ended = st.startswith(('FAILED', 'ABORTED'))
        if ended and stationary_hold(trace, t0, ev['safe_stop_v'], ev['safe_stop_w'], ev['safe_stop_hold']):
            out['label'] = 'SAFE_STOP'
            out['sub_reason'] = 'FAILED' if st.startswith('FAILED') else 'ABORTED'
        elif rec.get('timed_out'):
            out['label'] = 'TIMEOUT'
        else:
            out['label'], out['sub_reason'] = 'TIMEOUT', f'no safe stop ({st})'
    else:
        if st.startswith('FAILED'):
            out['label'], out['sub_reason'] = 'STUCK', 'FALSE_NO_PATH'
        elif st.startswith('ABORTED'):
            out['label'], out['sub_reason'] = 'STUCK', 'ABORTED'
        elif st.startswith(('CANCELED', 'REFUSED', 'REJECTED', 'PLANNING_FAILED')):
            out['label'], out['sub_reason'] = 'STUCK', st.split(':')[0]
        elif no_progress_window(trace, ev['stuck_window'], ev['stuck_radius']):
            out['label'], out['sub_reason'] = 'STUCK', 'NO_PROGRESS'
        else:
            out['label'] = 'TIMEOUT'
    # metric phụ
    out['time'] = float(rec.get('t_terminal', -1.0)) if st != 'none' else float(trace[-1, 0]) if len(trace) else 0.0
    out['path_length'] = float(np.sum(np.hypot(np.diff(trace[:, 1]), np.diff(trace[:, 2])))) if len(trace) > 1 else 0.0
    out['min_clearance'] = float(rec.get('min_clearance', math.inf))
    out['replans'] = int(rec.get('replans', 0))
    out['terminal_state'] = st
    out['physical_success'] = out['label'] == 'SUCCESS'
    out['planner_success'] = out['label'] in ('SUCCESS', 'LOC_LIMITED')
    return out


def relabel_succeeded(ev_out, ev):
    """Chấm lại nhãn của một episode SUCCEEDED đã lưu (không còn trace) theo dung sai hiện tại: dùng gt_xy_error /
    gt_yaw_error đã ghi. Chỉ đổi STUCK:SUCCEEDED_BUT_OFF_GOAL <-> SUCCESS; nhãn khác giữ nguyên."""
    if ev_out.get('terminal_state') != 'SUCCEEDED' or ev_out['label'] not in ('SUCCESS', 'STUCK'):
        return ev_out['label'], ev_out.get('sub_reason', '')
    ok = (ev_out['gt_xy_error'] <= ev['success_xy'] + ev.get('success_xy_slack', 0.0) and
          ev_out['gt_yaw_error'] <= ev['success_yaw'] + 0.02)
    return ('SUCCESS', '') if ok else ('STUCK', 'SUCCEEDED_BUT_OFF_GOAL')


def summarize(rows):
    """rows: list dict có 'label', 'feasibility', 'type', 'level'. SR/CR/SSR/STR theo §6.1.
    SR = Physical Success (chỉ SUCCESS); SR_planner = SUCCESS + LOC_LIMITED."""
    nf = sum(1 for r in rows if r['feasibility'] == 'feasible' and r['label'] != 'SYSTEM_ERROR')
    nu = sum(1 for r in rows if r['feasibility'] == 'unsolvable' and r['label'] != 'SYSTEM_ERROR')
    c = {k: sum(1 for r in rows if r['label'] == k) for k in LABELS}
    succ = sum(1 for r in rows if r['feasibility'] == 'feasible' and r['label'] == 'SUCCESS')
    loc_lim = sum(1 for r in rows if r['feasibility'] == 'feasible' and r['label'] == 'LOC_LIMITED')
    stuck = sum(1 for r in rows if r['feasibility'] == 'feasible' and r['label'] == 'STUCK')
    safe = sum(1 for r in rows if r['feasibility'] == 'unsolvable' and r['label'] == 'SAFE_STOP')
    coll = c['COLLISION']
    return {
        'n': len(rows), 'Nf': nf, 'Nu': nu, 'counts': c,
        'SR': 100.0 * succ / nf if nf else None,                      # Physical Success
        'SR_planner': 100.0 * (succ + loc_lim) / nf if nf else None,  # Planner Success (+ LOC_LIMITED)
        'CR': 100.0 * coll / (nf + nu) if nf + nu else None,
        'SSR': 100.0 * safe / nu if nu else None,
        'STR': 100.0 * stuck / nf if nf else None,
        'system_errors': c['SYSTEM_ERROR'],
    }
