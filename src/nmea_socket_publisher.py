#!/usr/bin/env python2
import socket
import rospy

from nmea_msgs.msg import Sentence

rospy.init_node('nmea_messages')
pub = rospy.Publisher('nmea_sentence', Sentence, queue_size=20)
r = rospy.Rate(20)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.0.101', 6002))

f = s.makefile()
while not rospy.is_shutdown():
    msg = Sentence()
    msg.sentence = f.readline()[:-2]
    msg.header.frame_id = 'gps'
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    r.sleep()

f.close()