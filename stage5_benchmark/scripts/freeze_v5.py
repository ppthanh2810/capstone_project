#!/usr/bin/env python3
"""Đóng băng cấu hình Navigation V5 dùng chung cho M0 (DWA tối ưu cố định) và M1 (SAC-DWA) — Master Plan v5 bước 8.

Ghi (--write) hoặc kiểm tra (--check) file config/navigation_v5_frozen.json gồm:
  - git commit + cờ dirty của repo, sha256 của nav2_params.yaml và benchmark.yaml;
  - sha256 mã nguồn điều hướng mà Fast 2D dùng lại (DwaCore, ReplanSupervisor, path_blockage, Fast 2D, ZlacDriverModel)
    và module Python fast2d đã build;
  - cấu hình đã nạp (C.load(): dwb / replan / sim / eval) để M1 dùng ĐÚNG cấu hình này;
  - hash dataset: sha256 của manifest.jsonl + mọi maps/*.npz của từng split v3 và stress_pilot.
--check trả mã 1 nếu có gì khác (M1 phải chạy --check trước khi training / đánh giá).

  freeze_v5.py --write [--out config/navigation_v5_frozen.json]
  freeze_v5.py --check
"""
import argparse
import glob
import hashlib
import json
import os
import subprocess
import sys

from stage5_benchmark import config as C

REPO = C.repo_root()
DATA = os.path.expanduser('~/capstone_ws/bench_data')
SOURCES = ['stage3_navigation/src/dwa_core.cpp', 'stage3_navigation/include/stage3_navigation/dwa_core.hpp',
           'stage3_navigation/src/replan_supervisor.cpp',
           'stage3_navigation/include/stage3_navigation/replan_supervisor.hpp',
           'stage3_navigation/src/path_blockage.cpp', 'stage3_navigation/src/goal_path_bridge.cpp',
           'stage5_benchmark/src/fast2d.cpp', 'stage5_benchmark/include/stage5_benchmark/fast2d.hpp',
           'stage5_benchmark/src/fast2d_py.cpp', 'stage5_benchmark/python/stage5_benchmark/evaluator.py',
           'stage5_benchmark/python/stage5_benchmark/runner.py', 'stage5_benchmark/python/stage5_benchmark/config.py']


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def split_hash(d):
    h = hashlib.sha256()
    files = [os.path.join(d, 'manifest.jsonl')] + sorted(glob.glob(os.path.join(d, 'maps', '*.npz')))
    for p in files:
        h.update(os.path.relpath(p, d).encode())
        h.update(sha(p).encode())
    n = sum(1 for _ in open(files[0]))
    return {'sha256': h.hexdigest(), 'n_scenarios': n, 'n_maps': len(files) - 1}


def snapshot():
    git = lambda *a: subprocess.run(['git', '-C', REPO, *a], capture_output=True, text=True).stdout.strip()  # noqa
    from stage5_benchmark import fast2d
    srcs = {}
    for rel in SOURCES:
        p = os.path.join(REPO, rel)
        srcs[rel] = sha(p) if os.path.exists(p) else None
    zlac = sorted(glob.glob(os.path.join(REPO, 'stage4_gazebo', '**', '*zlac_driver_model*'), recursive=True))
    for p in zlac:
        srcs[os.path.relpath(p, REPO)] = sha(p)
    cfg = C.load()
    datasets = {}
    for d in sorted(glob.glob(os.path.join(DATA, 'v3', '*'))) + [os.path.join(DATA, 'stress_pilot')]:
        if os.path.isdir(d) and os.path.exists(os.path.join(d, 'manifest.jsonl')):
            datasets[os.path.relpath(d, DATA)] = split_hash(d)
    summ = json.load(open(os.path.join(DATA, 'v3', 'summary.json')))
    return {
        'name': 'Navigation V5 (R2s Local Avoidance First + Slow Approach), SIM_READY',
        'git_commit': git('rev-parse', 'HEAD'),
        'git_dirty_paths': git('status', '--porcelain').splitlines(),
        'nav2_params_sha256': sha(C.nav2_params_path()),
        'benchmark_yaml_sha256': sha(C.benchmark_yaml_path()),
        'fast2d_module_sha256': sha(fast2d.__file__),
        'sources_sha256': srcs,
        'config': {k: cfg[k] for k in ('dwb', 'replan', 'sim', 'eval')},
        'dataset': {'root': DATA, 'generator_version': summ.get('generator_version'),
                    'config_hash': summ.get('config_hash'), 'splits': datasets},
        'locked_splits': sorted(k for k in datasets if k.endswith('_test')),
    }


def diff(a, b, path=''):
    out = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            out += diff(a.get(k), b.get(k), f'{path}.{k}' if path else k)
    elif a != b:
        out.append(f'{path}: {a!r} != {b!r}')
    return out


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--write', action='store_true')
    g.add_argument('--check', action='store_true')
    ap.add_argument('--out', default=os.path.join(REPO, 'stage5_benchmark', 'config', 'navigation_v5_frozen.json'))
    ap.add_argument('--ignore', default='git_commit,git_dirty_paths',
                    help='khóa bỏ qua khi --check (mặc định: commit đổi do chỉ thêm tài liệu / kết quả)')
    a = ap.parse_args()
    snap = snapshot()
    if a.write:
        snap['frozen_at'] = __import__('datetime').date.today().isoformat()
        json.dump(snap, open(a.out, 'w'), indent=1, ensure_ascii=False, default=float)
        print('wrote', a.out, 'commit', snap['git_commit'][:7], 'nav2_params', snap['nav2_params_sha256'][:12])
        return 0
    ref = json.load(open(a.out))
    ign = set(a.ignore.split(',')) | {'frozen_at'}
    d = diff({k: v for k, v in ref.items() if k not in ign}, {k: v for k, v in snap.items() if k not in ign})
    if d:
        print('FROZEN CONFIG MISMATCH:\n  ' + '\n  '.join(d[:40]))
        return 1
    print('frozen V5 config OK', ref['nav2_params_sha256'][:12], 'dataset splits', len(ref['dataset']['splits']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
