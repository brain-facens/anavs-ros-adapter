#!/usr/bin/env python2
"""Translates the information from ublox GNSS to pseudo-Mercator
"""

import numpy as np
import pyproj
import rospy
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3
from ublox_msgs.msg import NavPVT

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def combine(ubx_sol):
    """Combines orientation information from ENU message with position from LLH"""

    # Tranlates coordinates
    x, y = coord_transformer.transform(ubx_sol.lat * 1e-7, ubx_sol.lon * 1e-7)
    
    # Theta is half the heading angle
    theta = np.deg2rad(ubx_sol.heading * 1e-5) / 2

    new_tf = TransformStamped(
        header = Header(
            stamp = rospy.Time.now(),
            frame_id = "map"
        ),
        child_frame_id = "gps",
        transform = Transform(
            translation = Vector3(x, y, ubx_sol.height * 1e-3),
            rotation = Quaternion(w=np.cos(theta), z=np.sin(theta))
        )
    )

    br.sendTransform(new_tf)

rospy.init_node("custom_ublox")
br = tf2_ros.TransformBroadcaster()
rospy.Subscriber("ublox/navpvt", NavPVT, combine, queue_size=10)
rospy.spin()
