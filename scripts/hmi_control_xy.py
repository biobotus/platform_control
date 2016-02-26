#!/usr/bin/env python

# Imports
import rospy
from std_msgs.msg import Float32, Int32, String

class hmi():
    def __init__(self):
        self.delta_x = 0
        self.delta_y = 0

        self.pos_x = 0
        self.pos_y = 0
        self.run = 1

        # ROS publishments
        self.pub_pos_xy = rospy.Publisher('Pulse_XY', String, queue_size=10)
        self.pub_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.pub_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)

        # ROS init
        self.rate = rospy.Rate(10)  # 10Hz

    def hmi_control_pos(self):
        self.run = 1
        while self.run:

            print("\n Position control Axis")

            mm_x = float(raw_input(" Move X (mm) : "))
            mm_y = float(raw_input(" Move Y (mm) : "))

            self.delta_x = int(mm_x/(0.127*0.25))
            self.delta_y = int(mm_y/(0.127*0.25))

            msg = '[{0},{1}]'.format(self.delta_x, self.delta_y)
            print("Publishing {0}".format(msg))
            self.pub_pos_xy.publish(msg)

            self.run = int(raw_input(" Do you want to continue? (yes = 1/no = 0)"))
            self.rate.sleep()

            self.pub_move.publish('motor_control_x')

if __name__ == '__main__':
    rospy.init_node('hmi_control')

    h = hmi()

    try:
        h.hmi_control_pos()

    except rospy.ROSInterruptException as e:
        print(e)

    finally:
        # Kill motors
        h.pub_kill.publish("motor_control_x")
        h.pub_kill.publish("motor_control_y")

