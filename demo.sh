#!/bin/bash
# =============================================================
#  demo.sh -- lance tout le robot dans le bon ordre (salon IBT)
#
#  ./demo.sh          demarre tout dans une session tmux "robot"
#  ./demo.sh stop     arrete tout proprement
#  tmux attach -t robot   revenir dans la session (ex. apres une coupure SSH)
#
#  Dans tmux : Ctrl+B puis 0..5 = changer de fenetre, Ctrl+B puis d = quitter
#  la session SANS rien arreter.
#
#  Fenetres : 0 robot | 1 nav2 | 2 camera | 3 aruco | 4 mission (commande
#  deja tapee : il suffit d'appuyer sur Entree) | 5 libre
# =============================================================
SESSION=robot
CAM=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_09663B1F-video-index0
FOCUS=20          # meme valeur que pendant la calibration !
FACE_PRISE=0.29
NAV2_PARAMS=$HOME/ros2_ws/src/robot_bringup/config/nav2_params.yaml

source /opt/ros/jazzy/setup.bash
source $HOME/ros2_ws/install/setup.bash

# ---------------------------------------------------------------- arret
if [ "$1" = "stop" ]; then
  if ! tmux has-session -t $SESSION 2>/dev/null; then
    echo "Rien a arreter."; exit 0
  fi
  echo "Arret (Ctrl+C dans chaque fenetre)..."
  for w in mission aruco camera nav2 robot; do
    tmux send-keys -t $SESSION:$w C-c 2>/dev/null
    sleep 1
  done
  sleep 3
  tmux kill-session -t $SESSION
  echo "Tout est arrete."
  exit 0
fi

if tmux has-session -t $SESSION 2>/dev/null; then
  echo "Deja lance. Pour y revenir : tmux attach -t $SESSION"
  echo "Pour tout relancer       : ./demo.sh stop  puis  ./demo.sh"
  exit 1
fi

etape() { echo -e "\n\e[1;34m==> $1\e[0m"; }
ok()    { echo -e "\e[1;32m    OK\e[0m $1"; }
ko()    { echo -e "\e[1;31m    PROBLEME : $1\e[0m"; }

# ---------------------------------------------------------------- 0. robot
etape "Robot (lidar, odometrie, SLAM, TF, Foxglove)"
tmux new-session -d -s $SESSION -n robot
tmux send-keys -t $SESSION:robot "ros2 launch robot_bringup robot_slam.launch.py odom:=true foxglove:=true" C-m

echo "    attente de /odom..."
n=0
until timeout 3 ros2 topic echo /odom --once >/dev/null 2>&1; do
  n=$((n + 1))
  if [ $n -gt 20 ]; then
    ko "pas d'odometrie apres 60 s : regarder la fenetre 0 (ESP32 branche ? /dev/esp32 ?)"
    echo "    tmux attach -t $SESSION"
    exit 1
  fi
done
ok "/odom recu"

# ---------------------------------------------------------------- 1. Nav2
etape "Nav2"
tmux new-window -t $SESSION -n nav2
tmux send-keys -t $SESSION:nav2 "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$NAV2_PARAMS" C-m

# ---------------------------------------------------------------- 2. camera
etape "Camera"
tmux new-window -t $SESSION -n camera
tmux send-keys -t $SESSION:camera "ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=$CAM -p image_size:='[640,480]' -p camera_frame_id:=camera_optical -r __ns:=/camera" C-m
sleep 6
v4l2-ctl -d $CAM -c focus_automatic_continuous=0 && v4l2-ctl -d $CAM -c focus_absolute=$FOCUS
F=$(v4l2-ctl -d $CAM -C focus_absolute 2>/dev/null | awk '{print $2}')
A=$(v4l2-ctl -d $CAM -C focus_automatic_continuous 2>/dev/null | awk '{print $2}')
if [ "$F" = "$FOCUS" ] && [ "$A" = "0" ]; then ok "mise au point fixee a $FOCUS"; else ko "mise au point : focus=$F auto=$A (attendu $FOCUS / 0)"; fi

# ---------------------------------------------------------------- 3. aruco
etape "Detecteur ArUco"
tmux new-window -t $SESSION -n aruco
tmux send-keys -t $SESSION:aruco "ros2 run robot_bringup aruco_pose.py" C-m

# ---------------------------------------------------------------- attente Nav2
echo "    attente de Nav2 (jusqu'a 90 s)..."
n=0
until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; do
  n=$((n + 1))
  if [ $n -gt 45 ]; then
    ko "Nav2 n'est pas actif : regarder la fenetre 1, puis ./demo.sh stop et relancer"
    break
  fi
  sleep 2
done
[ $n -le 45 ] && ok "Nav2 actif"

# ---------------------------------------------------------------- 4. mission (prete, pas lancee)
tmux new-window -t $SESSION -n mission
tmux send-keys -t $SESSION:mission "ros2 run robot_bringup mission_brique.py --ros-args -p face_prise:=$FACE_PRISE"
tmux new-window -t $SESSION -n libre

echo -e "\n\e[1;32mPRET.\e[0m Brique en place (tag vers le robot) ?"
echo "  -> fenetre 4 (mission) : appuyer sur Entree pour lancer la mission"
echo "  -> pour recommencer : ./demo.sh stop, replacer le robot, ./demo.sh"
sleep 2
tmux select-window -t $SESSION:mission
if [ -n "$TMUX" ]; then tmux switch-client -t $SESSION; else tmux attach -t $SESSION; fi
