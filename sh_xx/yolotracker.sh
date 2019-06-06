#zhi qi dong tracker cheng xu
source /home/nvidia/chuan_cv/usv_ImagePro_v0.2/devel/setup.bash
echo "run  YOLO tracker on GPU..."
roslaunch yolo_tracker yolotracker.launch
sleep 1
wait
exit 0
