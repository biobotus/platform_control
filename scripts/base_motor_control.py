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
        self.gpio = pigpio.pi()
        for A in self.mode:
            if self.sync[A]:
                for B in range(self.sync[A]):
                    self.gpio.set_mode(self.enable_pin[A][B], pigpio.OUTPUT)
                    self.gpio.set_mode(self.limit_sw_init[A][B], pigpio.INPUT)
                    self.gpio.set_mode(self.limit_sw_end[A][B], pigpio.INPUT)
                    self.gpio.set_mode(self.dir_pin[A][B], pigpio.OUTPUT)

                    # GPIO output init
                    self.gpio.write(self.enable_pin[A][B], pigpio.LOW)
                    self.gpio.write(self.dir_pin[A][B], self.direction[A])
                self.gpio.set_mode(self.clock_pin[A], pigpio.OUTPUT)
                #self.gpio.set_mode(self.dir_pin[A], pigpio.OUTPUT)

                # GPIO output init
                self.gpio.write(self.clock_pin[A], pigpio.LOW)
                #self.gpio.write(self.dir_pin[A], self.direction[A])

            else:
                self.gpio.set_mode(self.enable_pin[A], pigpio.OUTPUT)
                self.gpio.set_mode(self.clock_pin[A], pigpio.OUTPUT)
                self.gpio.set_mode(self.dir_pin[A], pigpio.OUTPUT)
                self.gpio.set_mode(self.limit_sw_init[A], pigpio.INPUT)
                self.gpio.set_mode(self.limit_sw_end[A], pigpio.INPUT)

                # GPIO output init
                self.gpio.write(self.enable_pin[A], pigpio.LOW)
                self.gpio.write(self.clock_pin[A], pigpio.LOW)
                self.gpio.write(self.dir_pin[A], self.direction[A])

    def callback_kill(self, data):
        if data.data == self.node_name:
            print("Killing {0}".format(self.node_name))
            for A in self.mode:
                if self.sync[A]:
                    for B in range(self.sync[A]):
                        self.gpio.write(self.enable_pin[A][B], pigpio.LOW)
                else:
                self.gpio.write(self.enable_pin[A], pigpio.LOW)

    def callback_init(self, data):
        if data.data:
            for A in self.mode:
                sleep_time = 0.125/self.f_min[A]

                # Set direction towards home
                self.gpio.write(self.dir_pin[A], pigpio.LOW)

                # Enable motion
                self.gpio.write(self.enable_pin[A], pigpio.HIGH)

                # TODO - Add sync condition
                # Initialization
                while not self.gpio.read(self.limit_sw[A]):
                    self.gpio.write(self.clock_pin[A], pigpio.HIGH)
                    start_time = time.clock()
                    while(time.clock() - start_time < sleep_time):
                        pass
                    self.gpio.write(self.clock_pin[A], pigpio.LOW)
                    start_time = time.clock()
                    while(time.clock() - start_time < sleep_time):
                        pass

                # Disable motion
                self.gpio.write(self.enable_pin[A], pigpio.LOW)

    # Listening function
    def listener(self):
        rospy.spin()

