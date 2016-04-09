#!/usr/bin/python

# Imports
import pigpio
import rospy
from std_msgs.msg import Int32, String

class GlobalMotorControl:

    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        # Expected format of Global_GPIO is 0 or 1
        self.subscriber = rospy.Subscriber('Global_GPIO', Int32, self.callback_gpio)

        # ROS publishments
        self.pub_sw = rospy.Publisher('Global_SW', String, queue_size=10)

        # GPIO pins
        self.global_enable_pin = 4   # pin 7
        self.global_limit_sw   = 18  # pin 12

        self.gpio = pigpio.pi()
        # self.gpio2 = pigpio.pi('10.43.156.12', 8888)
        self.gpio.set_mode(self.global_enable_pin, pigpio.OUTPUT)
        # self.gpio2.set_mode(self.global_enable_pin, pigpio.OUTPUT)
        self.gpio.write(self.global_enable_pin, pigpio.LOW)
        # self.gpio2.write(self.global_enable_pin, pigpio.LOW)
        self.gpio.set_mode(self.global_limit_sw, pigpio.INPUT)
        cb_sw = self.gpio.callback(self.global_enable_pin, pigpio.RISING_EDGE, \
                                                               self.callback_sw)

    def __del__(self):
        if hasattr(self.__class__, 'gpio'):
            print("Global enable pin = HIGH")
            self.gpio.write(self.global_enable_pin, pigpio.HIGH)

    def callback_gpio(self, data):
        if data.data == 0:
            self.gpio.write(self.global_enable_pin, pigpio.LOW)
        else:
            self.gpio.write(self.global_enable_pin, pigpio.HIGH)

    def callback_sw(self):
        self.gpio.write(self.global_enable_pin, pigpio.HIGH)
        self.pub_sw.publish(self.node_name)

    def listener(self):
        rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        gmc = GlobalMotorControl()
        gmc.listener()

    except rospy.ROSInterruptException as e:
        print(e)

