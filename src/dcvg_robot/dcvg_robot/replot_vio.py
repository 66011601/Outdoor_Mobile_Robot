"""
replot_vio.py

Re-generates route and error plots from CSV files produced by
geometric_route_runner or recorded_route_runner.

Single file — saves route + error plots next to the CSV:
  python3 replot_vio.py route_RouteA_20260610.csv

Multiple files — saves individual route plots AND one combined error
overlay so you can compare all scenarios side-by-side:
  python3 replot_vio.py route_RouteA_*.csv

Options:
  --outdir DIR    Save output here instead of next to the CSV files
  --labels A,B,C  Override legend labels (comma-separated, one per file)
  --ymax N        Error plot y-axis max in metres (default 100)
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ── plot style ────────────────────────────────────────────────────────────────
ROUTE_FIGSIZE = (10, 8)
ERROR_FIGSIZE = (10, 6)
DPI           = 150
ERROR_YMIN    = 0.0
ERROR_YMAX    = 100.0   # fixed — change with --ymax

COLOR_GT      = '#2ecc71'
COLOR_VIO     = '#e74c3c'
COLOR_START   = '#2c3e50'
GRID_ALPHA    = 0.35

# Colours for multi-file overlay (cycles if >8 files)
MULTI_COLORS  = [
    '#e74c3c', '#3498db', '#e67e22', '#9b59b6',
    '#1abc9c', '#f39c12', '#2ecc71', '#e91e63',
]

FONT_TITLE    = 14
FONT_AX       = 12


# ── CSV loading ───────────────────────────────────────────────────────────────

def load_csv(path: str) -> dict:
    """Return dict with arrays: time, gt_x, gt_y, vo_x, vo_y, error."""
    data = np.genfromtxt(path, delimiter=',', names=True)
    return {
        'time':  data['time'],
        'gt_x':  data['gt_x'],
        'gt_y':  data['gt_y'],
        'vo_x':  data['vo_x'],
        'vo_y':  data['vo_y'],
        'error': data['error'],
        'path':  path,
    }


def stats(d: dict) -> tuple[float, float, float]:
    e = d['error']
    return float(np.mean(e)), float(np.sqrt(np.mean(e**2))), float(np.max(e))


# ── single-file plots ─────────────────────────────────────────────────────────

def plot_route(d: dict, label: str, outdir: str, stamp: str) -> str:
    fig, ax = plt.subplots(figsize=ROUTE_FIGSIZE)
    fig.suptitle(f'VIO Accuracy — {label}', fontsize=FONT_TITLE, fontweight='bold')

    ax.plot(d['gt_x'], d['gt_y'], color=COLOR_GT, lw=2.5,
            label='Ground truth', zorder=3)
    ax.plot(d['vo_x'], d['vo_y'], color=COLOR_VIO, lw=2.0, ls='--',
            label='VIO (/vo)', zorder=2)
    ax.scatter([0], [0], color=COLOR_START, s=80, zorder=5, label='Start')

    mean_e, rmse, max_e = stats(d)
    ax.set_title(
        f'2D Trajectory   mean={mean_e:.2f} m  RMSE={rmse:.2f} m  max={max_e:.2f} m',
        fontsize=FONT_AX)
    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)')
    ax.legend();  ax.grid(True, alpha=GRID_ALPHA);  ax.set_aspect('equal')
    plt.tight_layout()

    out = os.path.join(outdir, f'route_{label}_{stamp}.png')
    fig.savefig(out, dpi=DPI, bbox_inches='tight')
    plt.close(fig)
    return out


def plot_error_single(d: dict, label: str, outdir: str, stamp: str,
                      ymax: float) -> str:
    mean_e, rmse, max_e = stats(d)

    fig, ax = plt.subplots(figsize=ERROR_FIGSIZE)
    fig.suptitle(f'VIO Position Error — {label}',
                 fontsize=FONT_TITLE, fontweight='bold')

    ax.plot(d['time'], d['error'], color='#3498db', lw=1.5,
            label='Position error')
    ax.axhline(mean_e, color='#e67e22', ls='--', lw=1.5,
               label=f'Mean  {mean_e:.2f} m')
    ax.axhline(rmse,   color='#c0392b', ls=':',  lw=1.5,
               label=f'RMSE  {rmse:.2f} m')

    ax.set_xlabel('Time (s)');  ax.set_ylabel('Error (m)')
    ax.set_title(f'max={max_e:.2f} m', fontsize=FONT_AX)
    ax.set_ylim(ERROR_YMIN, ymax)
    ax.legend();  ax.grid(True, alpha=GRID_ALPHA)
    plt.tight_layout()

    out = os.path.join(outdir, f'error_{label}_{stamp}.png')
    fig.savefig(out, dpi=DPI, bbox_inches='tight')
    plt.close(fig)
    return out


# ── multi-file combined error overlay ────────────────────────────────────────

def plot_error_combined(datasets: list[dict], labels: list[str],
                        outdir: str, stamp: str, ymax: float) -> str:
    fig, ax = plt.subplots(figsize=ERROR_FIGSIZE)
    fig.suptitle('VIO Position Error — All Scenarios',
                 fontsize=FONT_TITLE, fontweight='bold')

    for i, (d, label) in enumerate(zip(datasets, labels)):
        color    = MULTI_COLORS[i % len(MULTI_COLORS)]
        mean_e, rmse, _ = stats(d)
        ax.plot(d['time'], d['error'], color=color, lw=1.5,
                label=f'{label}  (mean={mean_e:.2f} m  RMSE={rmse:.2f} m)',
                alpha=0.9)

    ax.set_xlabel('Time (s)');  ax.set_ylabel('Error (m)')
    ax.set_title('Comparison across scenarios', fontsize=FONT_AX)
    ax.set_ylim(ERROR_YMIN, ymax)
    ax.legend(fontsize=9);  ax.grid(True, alpha=GRID_ALPHA)
    plt.tight_layout()

    out = os.path.join(outdir, f'error_comparison_{stamp}.png')
    fig.savefig(out, dpi=DPI, bbox_inches='tight')
    plt.close(fig)
    return out


# ── entry point ───────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Re-plot VIO accuracy graphs from CSV files.')
    parser.add_argument('csv_files', nargs='+',
                        help='CSV file(s) from geometric_route_runner / '
                             'recorded_route_runner')
    parser.add_argument('--outdir',
                        help='Output directory (default: same as CSV file)')
    parser.add_argument('--labels',
                        help='Comma-separated labels, one per file '
                             '(default: filename without extension)')
    parser.add_argument('--ymax', type=float, default=ERROR_YMAX,
                        help=f'Error plot y-axis max in metres (default {ERROR_YMAX})')
    args = parser.parse_args(argv)

    # Resolve labels
    auto_labels = [Path(f).stem for f in args.csv_files]
    if args.labels:
        custom = [l.strip() for l in args.labels.split(',')]
        labels = (custom + auto_labels[len(custom):])[:len(args.csv_files)]
    else:
        labels = auto_labels

    # Resolve output directory
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Load all CSVs
    datasets = []
    for path in args.csv_files:
        if not os.path.isfile(path):
            print(f'ERROR: file not found: {path}', file=sys.stderr)
            sys.exit(1)
        try:
            datasets.append(load_csv(path))
        except Exception as e:
            print(f'ERROR loading {path}: {e}', file=sys.stderr)
            sys.exit(1)

    # Generate plots
    for d, label in zip(datasets, labels):
        outdir = args.outdir or os.path.dirname(os.path.abspath(d['path']))
        Path(outdir).mkdir(parents=True, exist_ok=True)

        r = plot_route(d, label, outdir, stamp)
        e = plot_error_single(d, label, outdir, stamp, args.ymax)
        mean_e, rmse, max_e = stats(d)
        print(f'[{label}]  mean={mean_e:.3f} m  RMSE={rmse:.3f} m  '
              f'max={max_e:.3f} m')
        print(f'  {r}')
        print(f'  {e}')

    # Combined overlay when more than one file
    if len(datasets) > 1:
        outdir = args.outdir or os.path.dirname(
            os.path.abspath(datasets[0]['path']))
        combined = plot_error_combined(datasets, labels, outdir, stamp,
                                       args.ymax)
        print(f'\nCombined comparison:')
        print(f'  {combined}')


if __name__ == '__main__':
    main()
