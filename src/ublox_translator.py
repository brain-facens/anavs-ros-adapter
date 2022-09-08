#!/usr/bin/env python3
"""Translates the information from ublox GNSS to pseudo-Mercator
"""

import numpy as np
import pyproj
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from ublox_msgs.msg import NavPOSLLH, NavVELNED

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def combine(enu_pose: NavVELNED, llh_position: NavPOSLLH):
    """Combines orientation information from ENU message with position from LLH

    :param pose: ENU pose message
    :type pose: PoseWithCovarianceStamped
    :param position: LLH position message
    :type position: PointStamped
    """

    # Creates a pose object and fills header
    new_pose = PoseStamped()
    new_pose.header = rospy.Time.now()

    # Creates Quaternion from heading information
    theta = np.deg2rad(enu_pose.heading * 1e-5) / 2
    new_pose.pose.orientation = Quaternion(w=np.cos(theta), z=np.sin(theta))
    
    # Tranlates coordinates and adds position information from LLH
    x, y = coord_transformer.transform(llh_position.lat * 1e-7, llh_position.lon * 1e-7)
    new_pose.pose.position = Point(x, y, llh_position.height * 1e-3)
