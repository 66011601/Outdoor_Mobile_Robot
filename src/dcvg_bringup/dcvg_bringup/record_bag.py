import argparse
import datetime
import os
import shutil
import signal
import subprocess
import sys
import time

CORE = [
    '/tf', '/tf_static', '/rosout', '/diagnostics', '/joint_states',
    '/camera/imu', '/imu/data', '/imu/data_throttled',
    '/camera/depth/camera_info', '/camera/infra1/camera_info', '/camera/infra2/camera_info',
    '/gps/fix', '/gps/vel', '/fix', '/fix_velocity', '/navpvt', '/navrelposned',
    '/gps/heading/navrelposned', '/gps/heading/imu',
    '/vo', '/odometry/local', '/odometry/global', '/odometry/gps', '/gps/filtered',
    '/camera/depth/obstacles', '/camera/depth/obstacles_raw', '/camera/depth/ground',
    '/local_costmap/costmap', '/local_costmap/published_footprint',
    '/global_costmap/costmap', '/global_costmap/published_footprint',
    '/plan', '/received_global_plan', '/lookahead_point', '/lookahead_collision_arc',
    '/behavior_tree_log', '/cmd_vel_nav', '/cmd_vel',
    '/teensy/feedback', '/dcvg/stop_reached', '/dcvg/stop_confirm', '/dcvg/telemetry', '/dcvg/reading',
]
DEPTH = ['/camera/depth/image_rect_raw']
STEREO = ['/camera/infra1/image_rect_raw', '/camera/infra2/image_rect_raw']
PROFILES = {
    'nav': CORE,
    'obstacles': CORE + DEPTH,
    'full': CORE + DEPTH + STEREO,
}
SPLIT_BYTES = 1_000_000_000
STATUS_PERIOD_S = 10.0


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def free_gb(path):
    return shutil.disk_usage(path).free / 1e9


def main():
    parser = argparse.ArgumentParser(prog='record_bag')
    parser.add_argument('profile', nargs='?', default='obstacles', choices=sorted(PROFILES))
    parser.add_argument('-o', '--out-dir', default=os.path.expanduser('~/bags'))
    parser.add_argument('-n', '--name', default='')
    parser.add_argument('--min-free-gb', type=float, default=5.0)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if free_gb(args.out_dir) < args.min_free_gb:
        print(f'Only {free_gb(args.out_dir):.1f} GB free in {args.out_dir} '
              f'(--min-free-gb {args.min_free_gb}). Not recording.')
        sys.exit(1)

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    label = f'{args.profile}_{args.name}_{stamp}' if args.name else f'{args.profile}_{stamp}'
    path = os.path.join(args.out_dir, label)
    cmd = ['ros2', 'bag', 'record', '-o', path,
           '--max-bag-size', str(SPLIT_BYTES),
           '--compression-mode', 'message', '--compression-format', 'zstd',
           '--compression-queue-size', '500', '--compression-threads', '4',
           *PROFILES[args.profile]]

    print(f'Recording profile "{args.profile}" to {path}. Ctrl-C to stop.', flush=True)
    proc = subprocess.Popen(cmd, start_new_session=True)
    start = time.monotonic()
    last_status = start
    try:
        while proc.poll() is None:
            time.sleep(1.0)
            now = time.monotonic()
            if now - last_status < STATUS_PERIOD_S:
                continue
            last_status = now
            elapsed = now - start
            size_mb = dir_bytes(path) / 1e6
            rate = size_mb / elapsed
            free = free_gb(args.out_dir)
            left_min = (free - args.min_free_gb) * 1e3 / rate / 60.0 if rate > 0 else float('inf')
            print(f'{elapsed / 60.0:5.1f} min  {size_mb / 1e3:6.2f} GB  {rate:5.1f} MB/s  '
                  f'{free:5.1f} GB free  ~{left_min:.0f} min left', flush=True)
            if free < args.min_free_gb:
                print(f'Free space below {args.min_free_gb} GB, stopping.', flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    print(f'Saved {path} ({dir_bytes(path) / 1e9:.2f} GB, {(time.monotonic() - start) / 60.0:.1f} min)')


if __name__ == '__main__':
    main()
