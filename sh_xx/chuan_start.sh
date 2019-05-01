#! /bin/bash

exit_()
{
    exit    
}
#检测网络连接
while true
do
ping -c 3 -i 0.2 -W 3 192.168.1.100 &> /dev/null
#ping -c 3 -i 0.2 -W 3 192.168.1.50 &> /dev/null
if [ $? -eq 0 ]
then
    echo "检测网络正常"
    break
else
    echo "检测网络连接异常"
fi
trap exit_ int
done


sleep 5
source /opt/ros/kinetic/setup.bash
source  ~/.bashrc
export ROS_MASTER_URI=http://control:11311
export ROS_HOSTNAME=control
source /home/nvidia/chuan/chuan_ws/devel/setup.bash

roslaunch chuan_control chuan_start.launch

#wait
#exit 0
