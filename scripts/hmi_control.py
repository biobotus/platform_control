#!/usr/bin/env python

# Imports
import rospy
from std_msgs.msg import Float32, String
#import RPi.GPIO as GPIO


class hmi():
    def __init__(self):
        self.delta_x = 0
        self.delta_y = 0

        self.vel_x = 0
        self.vel_y = 0

        self.pos_x = 0
        self.pos_y = 0
        self.run = 1

        # ROS publishments
        self.pub_pos_x = rospy.Publisher('X_Pos', Float32, queue_size = 10)
        self.pub_pos_y = rospy.Publisher('Y_Pos', Float32, queue_size = 10)
        self.pub_vel_x = rospy.Publisher('X_Vel', Float32, queue_size = 10)
        self.pub_vel_y = rospy.Publisher('Y_Vel', Float32, queue_size = 10)
        self.pub_kill = rospy.Publisher('Motor_Kill', String, queue_size = 10)

        #ROS init
        self.rate = rospy.Rate(10) # 10Hz

    def hmi_control_pos(self):
        self.run = 1
        while self.run:

            print "\n X Position : %s" %self.pos_x
            print " Y Position : %s" %self.pos_y

            print("\n Position control Axis")

            self.delta_x = input(" Move X (mm) : ")
            self.delta_y = input(" Move Y (mm) : ")
            self.pos_x = self.pos_x + self.delta_x
            self.pos_y = self.pos_y + self.delta_y

            self.pub_pos_x.publish(self.delta_x)
            self.pub_pos_y.publish(self.delta_y)

            self.run = input(" Do you want to continue? (yes = 1/no = 0)")
            self.rate.sleep()

    def hmi_control_vel(self):
        self.run = 1
        while self.run:
            print("\n Velocity  control Axis")

            self.vel_x = input(" Vel X (Hz) : ")
            self.vel_y = input(" Vel Y (Hz) : ")

            self.pub_vel_x.publish(self.vel_x)
            self.pub_vel_y.publish(self.vel_y)

            self.run = input(" Do you want to continue? (yes = 1/no = 0)")
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('hmi_control')

    try:
        h = hmi()
        p_v  = input(" Position or velocity control? (1 = pos/0 = vel) ")

        if p_v == 1:
            h.hmi_control_pos()
        else:
            h.hmi_control_vel()

    except rospy.ROSInterruptException as e:
        print(e)
        pass

    finally:
        # Kill motors
        h.pub_vel_x.publish(0)
        h.pub_vel_y.publish(0)
        h.pub_kill.publish("motor_control_x")
        h.pub_kill.publish("motor_control_y")
