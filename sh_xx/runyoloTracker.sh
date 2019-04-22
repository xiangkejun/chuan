sleep 1
source /home/nvidia/chuan_cv/usv_ImagePro_v0.2/devel/setup.bash
echo "run  YOLO tracker on GPU..."
roslaunch yolo_tracker runYoloTracker.launch

wait
exit 0
