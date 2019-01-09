^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gps_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2017-05-08)
------------------
* removed unused variables
* Contributors: Frank Hoeller

0.1.8 (2016-10-31)
------------------
* Fixing orientation in UTM odometry message
  Before the fix, the orientation in the odometry message was set to x = 1, y = 0, z = 0, w = 0. This corresponds to Euler angles of roll = pi, pitch = 0, yaw = 0. I believe the intent was for the orientation to be the identity, which is x = 0, y = 0, z = 0, w = 1.
* Contributors: Tom Moore

0.1.6
-----
* Initial catkin release
