# system/ — boot-time and system-level scripts

This folder holds scripts/unit files that run outside the ROS2 workspace
build (systemd units, shutdown helpers, etc.), kept here just so they're
version-controlled next to the robot code instead of living only in
`/etc/systemd/system/`.

## dcvg-osm-route-app.service

Added 2026-08-26. Autostarts the route-control web app
(`ros2 run dcvg_robot osm_route_app`, Flask on port 5000) on every boot.

This is a **copy for reference/version control** — the file that actually
controls boot behavior is the installed copy at
`/etc/systemd/system/dcvg-osm-route-app.service`. If you edit this repo copy,
re-install it:

```bash
sudo cp system/dcvg-osm-route-app.service /etc/systemd/system/dcvg-osm-route-app.service
sudo systemctl daemon-reload
sudo systemctl restart dcvg-osm-route-app.service
```

Useful commands:
```bash
systemctl status dcvg-osm-route-app.service --no-pager   # check it's running
journalctl -u dcvg-osm-route-app.service -f               # tail logs
sudo systemctl disable --now dcvg-osm-route-app.service   # turn off autostart
```

## Pre-existing `dcvg.service` (NOT created by this change)

There is also `/etc/systemd/system/dcvg.service`, already enabled/running
before this change. It launches an unrelated **legacy** script —
`/home/jetson/Desktop/DCVG_Robot/main/main_gps.py` — from before this ROS2
rebuild (old Arduino Mega + non-ROS2 camera code, no relation to
`dcvg_bringup`/`dcvg_robot`). As of 2026-08-26 it was looping errors (camera
not found, `/dev/arduino_mega` missing) while consuming ~1GB RAM. Left alone
since it wasn't part of this task — flagged here as a candidate to disable:

```bash
sudo systemctl disable --now dcvg.service
```

## shutdown.sh

Pre-existing (not part of this change). Thin wrapper: `exec sudo /usr/sbin/poweroff`.
