#!/usr/bin/env python2
import pyproj
import rospy
import tf2_ros
import message_filters as mf
import numpy as np

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3, TwistStamped, Quaternion
from sensor_msgs.msg import NavSatFix

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def quaternion_from_vector(vector):

    if vector.z != 0:
        rospy.logwarn('This node cannot deal with vectors with nonzero z component')
    
    return Quaternion(*quaternion_from_euler(0, 0, np.arctan2(vector.y, vector.x)))

def broadcast_gnss(fix, vel, gnss_name, br = tf2_ros.TransformBroadcaster()):
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
    if np.isnan(fix.altitude):
        rospy.logwarn('Missing altitude information')
        t.transform.translation = Vector3(x, y, 0)
    else:
        t.transform.translation = Vector3(x, y, fix.altitude)

    t.transform.rotation = quaternion_from_vector(vel.twist.linear)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('gnss_broadcaster')

    gnss_name = rospy.get_param('gnss_name', default='gps')
    fix_sub = mf.Subscriber('/fix', NavSatFix)
    vel_sub = mf.Subscriber('/vel', TwistStamped)

    ts = mf.TimeSynchronizer([fix_sub, vel_sub], 10)
    ts.registerCallback(broadcast_gnss, gnss_name)
    rospy.spin()
    