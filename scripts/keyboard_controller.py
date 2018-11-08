#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def pub(linear, angular):
    pub.publish(Twist(linear, angular))

def callback(data):
    def iskey(c):
        return data.data == ord(c)
    if iskey('w'):
        pub.publish(1.0, 0.0)
    elif iskey('s'):
        pub.publish(-1.0, 0.0)
    elif iskey('d'):
        pub.publish(0.3, 1.0)
    elif iskey('a'):
        pub.publish(1.0, 0.3)
    else:
        mc.send(0, 0)

if __name__ == '__main__':

    mc = LoCoMoCo()

    if not mc.isOpen():
        print('Unable to open serial port')
        exit(1)

    rospy.init_node('keyboard_controller', anonymous=True)

    rospy.Subscriber("keyboard", Int32, callback)
    pub = rospy.Publisher("/rb_drive/rb_drive/twist_cmd", Twist)

    rospy.spin()

    mc.close()
