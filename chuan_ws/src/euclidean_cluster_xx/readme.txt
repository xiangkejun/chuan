//xx
https://blog.csdn.net/AdamShan/article/details/83015570


注：
1.先编译jsk_recognition_msgs
2. 安装插件ros-kinetic-jsk-rviz-plugins

 

// roslaunch pcl_test  pcl_test.launch


roslaunch euclidean_cluster euclidean_cluster.launch

// 订阅点云数据后 得到box 以节点 /box_lidar_points发布

roslaunch euclidean_cluster box_lidar_rviz.launch

roslaunch euclidean_cluster box_lidar_xx.launch
