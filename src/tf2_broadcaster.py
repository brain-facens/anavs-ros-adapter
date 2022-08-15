#!/usr/bin/env python2
import pyproj
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import NavSatFix

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def broadcast_gnss(fix, gnss_name, br = tf2_ros.TransformBroadcaster()):
    """Broadcast GNSS data as TF2

    :param fix: Topic for GNSS data
    :type fix: str
    :param gnss_name: Name to represent that TF2
    :type gnss_name: str
    :param br: Broadcaster object (used to initialize it only once), defaults to tf2_ros.TransformBroadcaster()
    :type br: TransformBroadcaster, optional
    """
    t = TransformStamped()

    t.header.stamp = fix.header.stamp
    t.header.frame_id = "map"
    t.child_frame_id = gnss_name

    x, y = coord_transformer.transform(fix.latitude, fix.longitude)
    t.transform.translation = Vector3(x, y, fix.altitude)

    t.transform.rotation = Quaternion(w=1)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('gnss_broadcaster')
    gnss_name = rospy.get_param('gnss_name', default='gps')
    rospy.Subscriber('/fix', NavSatFix, broadcast_gnss, callback_args=gnss_name)
    rospy.spin()
    