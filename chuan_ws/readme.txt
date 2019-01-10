安装依赖：
sudo apt-get install ros-kinetic-jsk-footstep-msgs 
sudo apt-get install ros-kinetic-jsk-rviz-plugins

*******************************************************
船启动：
./chuan_start.sh 添加控制优先级
速度输入：                     优先级
cmd_vel_mux/input/teleop       8
cmd_vel_mux/input/cv_vel       7
cmd_vel_mux/input/xbox_joy     6
cmd_vel_mux/input/navi         5
速度输出：
mobile_base/commands/velocity
*********************************************
