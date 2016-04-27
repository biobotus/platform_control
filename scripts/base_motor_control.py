#!/usr/bin/python

# Imports
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
        self.done_module = rospy.Publisher('Done_Module', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

    def init_gpio(self):
        """This function initializes GPIO pins for the node"""

        self.gpio = pigpio.pi()
        self.gpio.wave_tx_stop()
        self.gpio.wave_clear()
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
            self.gpio.wave_tx_stop()
            self.gpio.wave_clear()

    def callback_init(self, data):
        if data.data != self.node_name:
            return

        print('Initializing {0}'.format(self.node_name))
        self.set_cb_sw()
        sw_or = 0
        active_gpios = 0
        microseconds = 1000000*0.5/self.f_init

        for A in self.modes:
            # Set direction towards home
            self.gpio.write(self.dir_pin[A], self.init_dir)

            # Enable motion
            for B in range(self.sync[A]):
                current = self.gpio.read(self.limit_sw[A][B])
                sw_or = sw_or or current
                if current:
                    self.gpio.write(self.enable_pin[A][B], pigpio.HIGH)
                    self.init_list.append('{0}{1}'.format(A, B))
                    print('List contains : {0}'.format(self.init_list))
                    active_gpios = active_gpios | (1 << self.clock_pin[A])
                else:
                    self.gpio.write(self.enable_pin[A][B], pigpio.LOW)

        try:
            # Start PWM Frequency
            if sw_or:
                self.gpio.wave_clear()
                wave = [pigpio.pulse(active_gpios, 0, microseconds),
                        pigpio.pulse(0, active_gpios, microseconds)]
                self.gpio.wave_add_generic(wave)
                wid = self.gpio.wave_create()
                self.gpio.wave_send_repeat(wid)

            while self.init_list:
                self.rate.sleep()

        finally:
            self.gpio.wave_tx_stop()
            self.gpio.wave_clear()
            print('Stopped wave on {0}'.format(self.node_name))

        print('Init of {0} done'.format(self.node_name))
        self.done_module.publish(self.node_name)

    # Listening function
    def listener(self):
        rospy.spin()

