#!/bin/bash
# Запуск QGroundControl + record.py + control.py + RViz2

cd ~/Firmware2 || exit 1
source install/setup.bash

# QGroundControl
/home/sed/Documents/QGroundControl-x86_64.AppImage &

# Recorder
gnome-terminal --title="Concept-VLA Record" -- bash -c '
    cd ~/Firmware2
    source install/setup.bash

    echo "=== RECORD ==="
    python3 /home/sed/Desktop/concept-vla/scripts/record.py

    echo
    echo "=== record.py завершён. Нажмите Enter для закрытия. ==="
    read
' &

# Control
gnome-terminal --title="Concept-VLA Control" -- bash -c '
    cd ~/Firmware2
    source install/setup.bash

    echo "=== CONTROL ==="
    python3 /home/sed/Desktop/concept-vla/scripts/control.py

    echo
    echo "=== control.py завершён. Нажмите Enter для закрытия. ==="
    read
' &

# Дадим MAVROS / GUI немного времени
sleep 1

# RViz остаётся главным блокирующим процессом этого launcher
rviz2 -d /home/sed/.rviz2/gazebo-raw.rviz
