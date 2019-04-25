#! /bin/bash

exit_()
{
    exit    
}
#检测网络连接
while true
do
#ping -c 3 -i 0.2 -W 3 192.168.1.150 &> /dev/null
ping -c 3 -i 0.2 -W 3 192.168.1.50 &> /dev/null
if [ $? -eq 0 ]
then
    echo "检测网络正常"
    break
else
    echo "检测网络连接异常"
fi
trap exit_ int
done

sleep 20
export ROS_MASTER_URI=http://control:11311
export ROS_HOSTNAME=tx2
source /home/nvidia/chuan_cv/usv_ImagePro_v0.2/devel/setup.bash

roslaunch ros_cv cv_usb_test.launch

#wait
#exit 0
