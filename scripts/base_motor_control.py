#!/usr/bin/python

# Imports
from biobot_ros_msgs.msg import HomingMsg
import pigpio
import rospy
from std_msgs.msg import String
import time

class BaseMotorControl:
    """This class defines methods that are used by all MotorControl nodes."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Motor_Kill', String, self.callback_kill)
        self.subscriber = rospy.Subscriber('Platform_Init', String, self.callback_init)

        # ROS publishments
        self.homing = rospy.Publisher('Homing', HomingMsg, queue_size=10)
        self.done_module = rospy.Publisher('Done_Module', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)


        self.subscriber = rospy.Subscriber('Homing_Done', String, self.callback_temp)
        self.homing_done = False


    def init_gpio(self):
        """This function initializes GPIO pins for the node"""

        self.gpio = pigpio.pi()
        for A in self.modes:
            for B in range(self.sync[A]):
                self.gpio.set_mode(self.enable_pin[A][B], pigpio.OUTPUT)
                self.gpio.write(self.enable_pin[A][B], pigpio.LOW)
                self.gpio.set_mode(self.limit_sw[A][B], pigpio.INPUT)

            self.gpio.set_mode(self.clock_pin[A], pigpio.OUTPUT)
            self.gpio.write(self.clock_pin[A], pigpio.LOW)
            self.gpio.set_mode(self.dir_pin[A], pigpio.OUTPUT)
            self.gpio.write(self.dir_pin[A], self.direction[A])

    def callback_kill(self, data):
        if data.data == self.node_name:
            print('Killing {0}'.format(self.node_name))
            for A in self.modes:
                for B in range(self.sync[A]):
                    self.gpio.write(self.enable_pin[A][B], pigpio.LOW)

    def callback_init(self, data):
        if data.data != self.node_name:
            return

        print('Initializing {0}'.format(self.node_name))
        self.homing_done = False

        for A in self.modes:
            # Set direction towards home
            self.gpio.write(self.dir_pin[A], self.init_dir)

        homing_msg = HomingMsg()
        homing_msg.ena_gpio = [i for j in self.enable_pin for i in j]
        homing_msg.clk_gpio = [self.clock_pin[i] for i in range(len(self.clock_pin)) \
                                            for j in range(len(self.enable_pin[i]))]
        homing_msg.sw_gpio  = [i for j in self.limit_sw for i in j]
        homing_msg.freq     = self.f_init

        self.homing.publish(homing_msg)

        while not self.homing_done:
            self.rate.sleep()

        print('Init of {0} done'.format(self.node_name))
        self.done_module.publish(self.node_name)

    def callback_temp(self, data):
        self.homing_done = True

    # Listening function
    def listener(self):
        rospy.spin()

