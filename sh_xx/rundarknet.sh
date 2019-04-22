sleep 1
source /home/nvidia/chuan_cv/usv_ImagePro_v0.2/devel/setup.bash
echo "run darknet..."
roslaunch darknet_ros yolo_v3-tiny.launch

wait
exit 0
