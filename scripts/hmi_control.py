#!/usr/bin/python

# Imports
from platform_control.msg import IntList
import rospy
from std_msgs.msg import Bool, String

class HMIControl():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS publishments
        self.pub_pos_xy = rospy.Publisher('Pulse_XY', IntList, queue_size=10)
        self.pub_pos_z = rospy.Publisher('Pulse_Z', IntList, queue_size=10)
        self.pub_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)
        self.pub_init = rospy.Publisher('Platform_Init', Bool, queue_size=10)

        # Variables
        self.delta_x = 0
        self.delta_y = 0
        self.pos_x = 0
        self.pos_y = 0
        self.run = 1

    def hmi_control_pos(self):
        self.run = 1
        init = int(raw_input(" Do you want to initialize the platform? \
                                                    (yes = 1/no = 0) "))

        if init:
            self.pub_init.publish(1)
            # while not self.init_done:
            #     self.rate.sleep()

        self.run = int(raw_input(" Move manually? (yes = 1/no = 0) "))

        while self.run:
            xy_or_z = int(raw_input(" Move XY (1) or Z(0)? "))
            if xy_or_z:
                print("\n Position Control Axis X and Y")
                mm_x = float(raw_input(" Move X (mm) : "))
                mm_y = float(raw_input(" Move Y (mm) : "))
                self.delta_x = int(mm_x/(0.127*0.25))
                self.delta_y = int(mm_y/(0.127*0.25))

                msg = IntList()
                msg.data = [self.delta_x, self.delta_y]
                print("Publishing {0}".format(msg.data))
                self.pub_pos_xy.publish(msg)

            else:
                print("\n Position Control Axis Z")
                ID = int(raw_input(" Move Z0 (0), Z1 (1) or Z2 (2)? "))
                mm_z = float(raw_input(" Move Z (mm) : "))
                self.delta_z = int(mm_z/(0.127*0.25))  # TODO use good values

                msg = IntList()
                msg.data = [ID, self.delta_z]
                print("Publishing {0}".format(msg.data))
                self.pub_pos_z.publish(msg)

            self.run = int(raw_input(" Move again? (yes = 1/no = 0)"))


if __name__ == '__main__':
    try:
        h = HMIControl()
        h.hmi_control_pos()

    except (rospy.ROSInterruptException, EOFError) as e:
        print(e)

    finally:
        # Kill motors
        h.pub_kill.publish("MotorControlXY")
        h.pub_kill.publish("MotorControlZ")

