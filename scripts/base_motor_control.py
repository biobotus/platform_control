#!/usr/bin/python

# Imports
import pigpio
import rospy
from std_msgs.msg import Bool, String
import time

class BaseMotorControl():
    """This class defines methods that are used by all MotorControl nodes."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Motor_Kill', String, self.callback_kill)
        self.subscriber = rospy.Subscriber('Platform_Init', Bool, self.callback_init)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

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
            print("Killing {0}".format(self.node_name))
            for A in self.modes:
                for B in range(self.sync[A]):
                    self.gpio.write(self.enable_pin[A][B], pigpio.LOW)

    def callback_init(self, data):
        if not data.data:
            return

        for A in self.modes:
            sleep_time = 0.125/self.f_min[A]

            # Set direction towards home
            self.gpio.write(self.dir_pin[A], pigpio.LOW)

            # Enable motion
            for B in range(self.sync[A]):
                self.gpio.write(self.enable_pin[A][B], pigpio.HIGH)

            sw_or = self.check_sw_init(A)

            while sw_or:
                self.gpio.write(self.clock_pin[A], pigpio.HIGH)
                software_sleep(sleep_time)
                self.gpio.write(self.clock_pin[A], pigpio.LOW)
                software_sleep(sleep_time)

                sw_or = self.check_sw_init(A)

                # TODO - Add sync condition
                # # Initialization
                # while not self.gpio.read(self.limit_sw[A]):
                #     self.gpio.write(self.clock_pin[A], pigpio.HIGH)
                #     start_time = time.clock()
                #     while(time.clock() - start_time < sleep_time):
                #         pass
                #     self.gpio.write(self.clock_pin[A], pigpio.LOW)
                #     start_time = time.clock()
                #     while(time.clock() - start_time < sleep_time):
                #         pass

                # # Disable motion
                # for B in range(self.sync[A]):
                #     self.gpio.write(self.enable_pin[A], pigpio.LOW)

    def check_sw_init(self, A):
        """
        Disables enable_pin on any of the sync'd axis if its switch is 0.
        Returns 0 if all sync'd limit_sw pins are at 0, else returns 1.
        """

        sw_or = 0
        for B in range(self.sync[A]):
            sw = self.gpio.read(self.limit_sw[A][B])
            sw_or = sw_or or sw
            if not sw:
                self.gpio.write(self.enable_pin[A][B], pigpio.LOW)
        return sw_or

    # Listening function
    def listener(self):
        rospy.spin()


def software_sleep(sleep_time):
    start_time = time.clock()
    while(time.clock() - start_time < sleep_time):
        pass
