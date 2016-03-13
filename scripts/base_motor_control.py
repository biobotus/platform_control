#!/usr/bin/python

# Imports
import pigpio
import rospy
from std_msgs.msg import String

class BaseMotorControl():
    """This class defines methods that are used by all MotorControl nodes."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Motor_Kill', String, self.callback_kill)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

    def init_gpio(self):
        self.gpio = pigpio.pi()
        for A in self.mode:
            self.gpio.set_mode(self.enable_pin[A], pigpio.OUTPUT)
            self.gpio.set_mode(self.clock_pin[A], pigpio.OUTPUT)
            self.gpio.set_mode(self.dir_pin[A], pigpio.OUTPUT)
            #self.gpio.set_mode(self.limit_sw[A], pigpio.INPUT)

            # GPIO output init
            self.gpio.write(self.enable_pin[A], pigpio.HIGH)
            self.gpio.write(self.clock_pin[A], pigpio.LOW)
            self.gpio.write(self.dir_pin[A], self.direction[A])

        # GPIO input init
        #self.gpio.set_glitch_filter(self.limit_sw[X], 50)
        #self.gpio.set_glitch_filter(self.limit_sw[Y], 50)
        #self.gpio.callback(self.limit_sw[X], pigpio.RISING_EDGE, \
        #                                   self.callback_limit_sw_x)
        #self.gpio.callback(self.limit_sw[Y], pigpio.RISING_EDGE, \
        #                                   self.callback_limit_sw_y)

    # def callback_limit_sw_x(self, gpio, level, tick):
    #     self.gpio.write(self.enable_pin[X], pigpio.HIGH)
    #     print("Callback Limit Switch X")

    # def callback_limit_sw_y(self, gpio, level, tick):
    #     self.gpio.write(self.enable_pin[Y], pigpio.HIGH)
    #     print("Callback Limit Switch Y")

    def callback_kill(self, data):
        if data.data == self.node_name:
            print("Killing {0}".format(self.node_name))
            for A in self.mode:
                self.gpio.write(self.enable_pin[A], pigpio.HIGH)

    # Listening function
    def listener(self):
        rospy.spin()

