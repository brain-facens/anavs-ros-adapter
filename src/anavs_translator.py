#!/usr/bin/env python3
"""Translates the information from ANAVS GNSS to pseudo-Mercator
"""

import message_filters
import pyproj
import rospy
from geometry_msgs.msg import (Point, PointStamped, PoseStamped,
                               PoseWithCovarianceStamped)

WSG84 = 4326
XY_METERS = 3857

coord_transformer = pyproj.Transformer.from_crs(WSG84, XY_METERS)

def combine(enu_pose: PoseWithCovarianceStamped, llh_position: PointStamped):
    """Combines orientation information from ENU message with position from LLH

    :param pose: ENU pose message
    :type pose: PoseWithCovarianceStamped
    :param position: LLH position message
    :type position: PointStamped
    """

    # Creates a pose object and fills header
    new_pose = PoseStamped()
    new_pose.header = enu_pose.header

    new_pose.pose.orientation = enu_pose.pose.pose.orientation
    
    # Tranlates coordinates and adds position information from LLH
    point = llh_position.point
    x, y = coord_transformer.transform(point.x, point.y)
    new_pose.pose.position = Point(x, y, point.z)
    
    publisher.publish(new_pose)

rospy.init_node("custom_anavs")
publisher = rospy.Publisher("pose", PoseStamped)
enu_sub = message_filters.Subscriber("/anavs/solution/pose_enu", PoseWithCovarianceStamped)
llh_sub = message_filters.Subscriber("/anavs/solution/pos_llh", PointStamped)

ts = message_filters.TimeSynchronizer([enu_sub, llh_sub], queue_size=10)

ts.registerCallback(combine)

rospy.spin()
