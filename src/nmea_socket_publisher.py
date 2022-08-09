#!/usr/bin/env python2
import socket
import rospy

from nmea_msgs.msg import Sentence

rospy.init_node('nmea_messages')

IP_ADDRESS = rospy.get_param('ip_addr',  default='192.168.0.101')
PORT       = rospy.get_param('port',     default=6002           )
FRAME_ID   = rospy.get_param('frame_id', default='gps'          )

publisher = rospy.Publisher('nmea_sentence', Sentence, queue_size=5)
r = rospy.Rate(200)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, PORT))

f = s.makefile()
while not rospy.is_shutdown():
    msg = Sentence()
    msg.sentence = f.readline()[:-2]
    msg.header.frame_id = FRAME_ID
    msg.header.stamp = rospy.Time.now()
    publisher.publish(msg)
    r.sleep()

f.close()