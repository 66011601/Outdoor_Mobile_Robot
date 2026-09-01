#!/usr/bin/env python3
"""
plot_path_tracking.py — read path_tracking_logger CSVs and show which failure
mode dominates.

    python3 plot_path_tracking.py /tmp/path_tracking.csv
    python3 plot_path_tracking.py /tmp/path_tracking.csv --lag 0.6 --save out.png

Runs off-robot (needs matplotlib + numpy; the logger itself needs neither, so
the rover is not required to have them installed).

PANELS
  1  cross-track error vs time, with path-change markers
       continuous oscillation      -> RPP geometry / lookahead too short
       steps at costmap resolution -> jagged NavFn path
       steady one-sided bias on curves -> open-loop track slip
  2  commanded omega vs time, plus omega shifted by the actuation lag
       spikes at fixed spacing     -> waypoint dispatch
  3  omega vs cross-track error, coloured by time
       a tight loop = a limit cycle, i.e. sustained oscillation rather than a
       one-off recovery
  4  the actual XY path geometry with robot track overlaid
       shows the staircase directly instead of inferring it
  5  cross-track error amplitude spectrum vs distance travelled
       a sharp peak at the waypoint spacing is the dispatch signature; a broad
       peak at a shorter wavelength is the pursuit limit cycle

The lag shift matters: with ~600ms of mechanical breakaway and no encoders, an
omega command and the cross-track error at the SAME timestamp are not cause and
effect. Panel 2 draws both so the correlation is read against the shifted trace.
"""

import argparse
import csv
import math
import sys

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    sys.exit('needs matplotlib and numpy:  pip3 install matplotlib numpy')


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{path} is empty')
    out = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]
           if k not in ('path_seq', 'path_len', 'nearest_idx')}
    out['path_seq'] = np.array([int(r['path_seq']) for r in rows])
    out['t'] -= out['t'][0]
    return out


def load_paths(path):
    p = path.rsplit('.', 1)[0] + '_paths.csv'
    try:
        with open(p) as f:
            rows = list(csv.DictReader(f))
    except FileNotFoundError:
        return {}
    paths = {}
    for r in rows:
        paths.setdefault(int(r['path_seq']), []).append(
            (float(r['x']), float(r['y'])))
    return paths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--lag', type=float, default=0.6,
                    help='actuation lag [s] used to shift the command trace')
    ap.add_argument('--save')
    a = ap.parse_args()

    d = load(a.csv)
    paths = load_paths(a.csv)
    t, xte, wz, vx = d['t'], d['cross_track_error'], d['cmd_wz'], d['cmd_vx']

    # Spatial axis for the spectrum. Waypoints are spaced in metres, so the
    # dispatch signature only shows up against distance, not time.
    #
    # Use ALONG-PATH progress, not robot arc length. Robot arc length is
    # inflated by the very lateral oscillation being measured: a sinusoid of
    # amplitude 0.25m and wavelength 1.5m has an arc length ~1.28x its extent,
    # which stretches a 1.5m feature to ~1.9m and would read as 2m waypoint
    # dispatch when it is nothing of the kind. dist_to_path_end is measured
    # along the path and is immune to that, so it is the honest axis.
    # Progress is accumulated per path_seq, since each replan restarts it.
    along = np.zeros_like(t)
    offset = 0.0
    for s in np.unique(d['path_seq']):
        m = d['path_seq'] == s
        rem = d['dist_to_path_end'][m]
        local = rem.max() - rem            # 0 at segment start, grows forward
        along[m] = offset + local
        offset = along[m].max()
    # Fall back to arc length if dist_to_path_end was never populated.
    if along.max() <= 1e-6:
        along = np.concatenate([[0], np.cumsum(
            np.hypot(np.diff(d['robot_x']), np.diff(d['robot_y'])))])
    dist = along

    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.25)

    # 1 — cross-track error
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, xte, lw=1.0, color='tab:blue')
    ax1.axhline(0, color='k', lw=0.6)
    for s in np.unique(d['path_seq'])[1:]:
        ax1.axvline(t[np.argmax(d['path_seq'] == s)], color='tab:red',
                    ls=':', lw=0.8, alpha=0.7)
    rms = float(np.sqrt(np.mean(xte ** 2)))
    bias = float(np.mean(xte))
    ax1.set_title(f'1. cross-track error   RMS={rms:.3f} m   mean(bias)={bias:+.3f} m'
                  f'   [red dotted = replan]')
    ax1.set_ylabel('XTE [m]  (+ = left)')
    ax1.grid(alpha=0.3)
    if abs(bias) > 0.5 * rms and rms > 0.05:
        ax1.text(0.01, 0.05, 'one-sided bias -> suspect open-loop track slip '
                             '(not fixable in software)',
                 transform=ax1.transAxes, color='darkred', fontsize=9)

    # 2 — commanded omega, raw and lag-shifted
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, wz, lw=0.9, label='cmd omega', color='tab:orange')
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.05
    shift = int(round(a.lag / dt)) if dt > 0 else 0
    if shift:
        ax2.plot(t[shift:], wz[:-shift] if shift else wz, lw=0.9, alpha=0.65,
                 color='tab:green', label=f'cmd omega +{a.lag:.2f}s lag')
    ax2.plot(t, vx, lw=0.9, color='tab:gray', alpha=0.7, label='cmd vx')
    ax2.set_title('2. commanded velocity (NOT achieved — open loop)')
    ax2.set_xlabel('t [s]')
    ax2.set_ylabel('omega [rad/s] / vx [m/s]')
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    # 3 — phase portrait: a closed loop means a limit cycle
    ax3 = fig.add_subplot(gs[1, 1])
    sc = ax3.scatter(xte, wz, c=t, cmap='viridis', s=4)
    ax3.axhline(0, color='k', lw=0.6)
    ax3.axvline(0, color='k', lw=0.6)
    ax3.set_title('3. omega vs XTE (closed loop = sustained oscillation)')
    ax3.set_xlabel('XTE [m]')
    ax3.set_ylabel('omega [rad/s]')
    ax3.grid(alpha=0.3)
    fig.colorbar(sc, ax=ax3, label='t [s]')

    # 4 — geometry
    ax4 = fig.add_subplot(gs[2, 0])
    for seq, pts in sorted(paths.items()):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax4.plot(xs, ys, lw=0.8, alpha=0.5,
                 label='global path' if seq == 0 else None)
    ax4.plot(d['robot_x'], d['robot_y'], lw=1.6, color='tab:red', label='robot')
    ax4.set_title('4. geometry — staircase visible directly')
    ax4.set_xlabel('x [m]')
    ax4.set_ylabel('y [m]')
    ax4.axis('equal')
    ax4.legend(fontsize=8)
    ax4.grid(alpha=0.3)

    # 5 — spatial spectrum of XTE
    ax5 = fig.add_subplot(gs[2, 1])
    if dist[-1] > 1.0 and len(dist) > 32:
        grid = np.linspace(0, dist[-1], len(dist))
        xte_u = np.interp(grid, dist, xte)
        xte_u -= xte_u.mean()
        ds = grid[1] - grid[0]
        amp = np.abs(np.fft.rfft(xte_u * np.hanning(len(xte_u))))
        freq = np.fft.rfftfreq(len(xte_u), d=ds)      # cycles per metre
        with np.errstate(divide='ignore'):
            wl = 1.0 / freq
        m = (wl > 0.3) & (wl < 20)
        ax5.plot(wl[m], amp[m], lw=1.0, color='tab:purple')
        if m.any() and amp[m].size:
            peak = wl[m][int(np.argmax(amp[m]))]
            ax5.axvline(peak, color='tab:red', ls='--', lw=1.0)
            ax5.text(0.98, 0.92, f'peak at {peak:.2f} m',
                     transform=ax5.transAxes, ha='right', color='darkred')
            ax5.text(0.98, 0.83,
                     'near waypoint spacing (1-2m) -> DISPATCH\n'
                     'much shorter -> pursuit limit cycle',
                     transform=ax5.transAxes, ha='right', fontsize=8)
        ax5.set_xlabel('wavelength [m]')
        ax5.set_ylabel('|XTE| amplitude')
        ax5.set_title('5. XTE spectrum vs distance travelled')
        ax5.grid(alpha=0.3)
    else:
        ax5.text(0.5, 0.5, 'not enough travel for a spectrum',
                 ha='center', transform=ax5.transAxes)

    fig.suptitle(f'path tracking diagnostics — {a.csv}', fontsize=13)
    if a.save:
        fig.savefig(a.save, dpi=130, bbox_inches='tight')
        print(f'wrote {a.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
