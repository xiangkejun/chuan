#!/usr/bin/env python

# Translates from NavSatFix to GPSFix and back

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from gps_common.msg import GPSFix
from gps_common.msg import GPSStatus

navsat_pub = rospy.Publisher('navsat_fix_out', NavSatFix, queue_size=10)
gps_pub = rospy.Publisher('gps_fix_out', GPSFix, queue_size=10)

def navsat_callback(navsat_msg):
    gps_msg = GPSFix()
    gps_msg.header = navsat_msg.header
    gps_msg.status.status = navsat_msg.status.status

    gps_msg.status.motion_source = GPSStatus.SOURCE_NONE
    gps_msg.status.orientation_source = GPSStatus.SOURCE_NONE
    gps_msg.status.position_source = GPSStatus.SOURCE_NONE
    if ((navsat_msg.status.service & NavSatStatus.SERVICE_GPS) or 
        (navsat_msg.status.service & NavSatStatus.SERVICE_GLONASS) or
        (navsat_msg.status.service & NavSatStatus.SERVICE_GALILEO)):
        gps_msg.status.motion_source = gps_msg.status.motion_source | GPSStatus.SOURCE_GPS
        gps_msg.status.orientation_source = gps_msg.status.orientation_source | GPSStatus.SOURCE_GPS
        gps_msg.status.position_source = gps_msg.status.position_source | GPSStatus.SOURCE_GPS
    if (navsat_msg.status.service & NavSatStatus.SERVICE_COMPASS):
        gps_msg.status.orientation_source = gps_msg.status.orientation_source | GPSStatus.SOURCE_MAGNETIC

    gps_msg.latitude=navsat_msg.latitude
    gps_msg.longitude=navsat_msg.longitude
    gps_msg.altitude=navsat_msg.altitude
    gps_msg.position_covariance=navsat_msg.position_covariance
    gps_msg.position_covariance_type=navsat_msg.position_covariance_type
    gps_pub.publish(gps_msg)


# Translates from GPSFix to NavSatFix.
# As GPSFix can store much more information than NavSatFix, 
# a lot of this additional information might get lost.
def gps_callback(gps_msg):
    navsat_msg = NavSatFix()
    navsat_msg.header = gps_msg.header

    # Caution: GPSFix has defined some additional status 
    # constants, which are not defined in NavSatFix.
    navsat_msg.status.status=gps_msg.status.status

    navsat_msg.status.service = 0
    if (gps_msg.status.position_source & GPSStatus.SOURCE_GPS):
        navsat_msg.status.service = navsat_msg.status.service | NavSatStatus.SERVICE_GPS
    if (gps_msg.status.orientation_source & GPSStatus.SOURCE_MAGNETIC):
        navsat_msg.status.service = navsat_msg.status.service | NavSatStatus.SERVICE_COMPASS

    navsat_msg.latitude=gps_msg.latitude
    navsat_msg.longitude=gps_msg.longitude
    navsat_msg.altitude=gps_msg.altitude
    navsat_msg.position_covariance=gps_msg.position_covariance
    navsat_msg.position_covariance_type=gps_msg.position_covariance_type
    navsat_pub.publish(navsat_msg)

if __name__ == '__main__':
    rospy.init_node('fix_translator', anonymous=True)
    navsat_sub = rospy.Subscriber("navsat_fix_in", NavSatFix, navsat_callback)
    gps_sub = rospy.Subscriber("gps_fix_in", GPSFix, gps_callback)
    rospy.spin()
