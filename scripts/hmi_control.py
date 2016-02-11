#!/usr/bin/env python

# Imports
import rospy
from std_msgs.msg import Float32
#import RPi.GPIO as GPIO


class hmi():
    def __init__(self):
        self.delta_x = 0
        self.delta_y = 0
        self.pos_x = 0
        self.pos_y = 0
        self.run = 1

        self.pub_x = rospy.Publisher('X_Move', Float32, queue_size=10)
        self.pub_y = rospy.Publisher('Y_Move', Float32, queue_size=10)

        #ROS init
        self.rate = rospy.Rate(10) # 10Hz

    def hmi_control(self):
        while self.run:
            print("")
            print " X Position : %s" %self.pos_x
            print " Y Position : %s" %self.pos_y
            print("")
            print(" Control Axis")

            self.delta_x = input(" Move X (mm) : ")
            self.delta_y = input(" Move Y (mm) : ")
            self.pos_x = self.pos_x+self.delta_x
            self.pos_y = self.pos_y+self.delta_y

            self.pub_x.publish(self.delta_x)
            self.pub_y.publish(self.delta_y)

            self.run=input(" Do you want to continue? (yes = 1/ no = 0)")
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('hmi_control')

    try:
        h = hmi()
        h.hmi_control()

    except rospy.ROSInterruptException as e:
        print(e)
        pass
