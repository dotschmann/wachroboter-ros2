#!/usr/bin/env bash
set -u

pkill -TERM -f '[f]ake_robot_sim.py' 2>/dev/null || true
pkill -TERM -f '[w]achroboter_node' 2>/dev/null || true
pkill -TERM -f '[t]opic_monitor.py' 2>/dev/null || true
pkill -TERM -f '[r]viz_labels.py' 2>/dev/null || true
pkill -TERM -f '/nav2_map_server/[m]ap_server' 2>/dev/null || true
pkill -TERM -f '[r]viz2' 2>/dev/null || true

echo "Wachroboter demo processes stopped."
