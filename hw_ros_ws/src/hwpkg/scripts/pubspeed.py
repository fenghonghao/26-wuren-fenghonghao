#!/usr/bin/env python
import rospy
from hwpkg.msg import turtlespeed
def talker():
    rospy.init_node('pubspeed', anonymous=True)
    pub = rospy.Publisher('/pubspeed/turtlespeed', turtlespeed, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        turtle_speed = turtlespeed()
        turtle_speed.vel = 5.0
        rospy.loginfo(turtle_speed)
        pub.publish(turtle_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass