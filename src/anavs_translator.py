#!/usr/bin/env python3
"""Translates the information from ANAVS GNSS to pseudo-Mercator
"""

import message_filters as mf
import pyproj
import rospy
import tf2_ros
from geometry_msgs.msg import (PointStamped, PoseWithCovarianceStamped,
                               TransformStamped, Transform, Vector3)

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def combine(enu_pose: PoseWithCovarianceStamped, llh_position: PointStamped):
    """Combines orientation information from ENU message with position from LLH"""

    # Tranlates coordinates and adds position information from LLH
    x, y = coord_transformer.transform(llh_position.point.x, llh_position.point.y)

    new_tf = TransformStamped(
        header = enu_pose.header,
        transform = Transform(
            translation = Vector3(x, y, llh_position.point.z),
            orientation = enu_pose.pose.pose.orientation
        )
    )
    br.sendTransform(new_tf)
    
rospy.init_node("custom_anavs")
br = tf2_ros.TransformBroadcaster()
enu_sub = mf.Subscriber("/anavs/solution/pose_enu", PoseWithCovarianceStamped)
llh_sub = mf.Subscriber("/anavs/solution/pos_llh", PointStamped)

ts = mf.TimeSynchronizer([enu_sub, llh_sub], queue_size=10)

ts.registerCallback(combine)

rospy.spin()
