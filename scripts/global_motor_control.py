#!/usr/bin/python

# Imports
import os
import pigpio
import rospy
from std_msgs.msg import Bool, String

class GlobalMotorControl:
    def __init__(self, ID):
        self.node_name = self.__class__.__name__ + ID
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        # Expected format of Global_Enable is True or False
        self.subscriber = rospy.Subscriber('Global_Enable', Bool, self.callback_gpio)

        # ROS publishments
        self.pub_sw = rospy.Publisher('Global_SW', String, queue_size=10)

        # GPIO pins
        self.global_enable_pin =  4  # pin 7
        self.global_limit_sw   = 21  # pin 40

        self.gpio = pigpio.pi()
        self.gpio.set_mode(self.global_enable_pin, pigpio.OUTPUT)
        self.gpio.write(self.global_enable_pin, pigpio.LOW)
        self.gpio.set_mode(self.global_limit_sw, pigpio.INPUT)
        self.cb_sw_active = False

    def __del__(self):
        if hasattr(self.__class__, 'gpio'):
            self.gpio.write(self.global_enable_pin, pigpio.LOW)

    def callback_gpio(self, data):
        if data.data:
            self.gpio.write(self.global_enable_pin, pigpio.HIGH)
            print("Global chip enable")
        else:
            self.gpio.write(self.global_enable_pin, pigpio.LOW)
            self.cb_sw.cancel()
            self.cb_active = False
            print("Global chip disable")

    def callback_sw(self):
        self.cb_sw.cancel()
        self.gpio.write(self.global_enable_pin, pigpio.LOW)
        self.pub_sw.publish(self.node_name)
        print("Cancelling global switch callback")
        self.cb_active = False

    def listener(self):
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
                if not self.cb_active and self.gpio.read(self.global_limit_sw):
                    self.cb_active = True
                    print("Creating global switch callback")
                    self.cb_sw = self.gpio.callback(self.global_limit_sw, \
                                                    pigpio.FALLING_EDGE, \
                                                    self.callback_sw)

        finally:
            del self


# Main function
if __name__ == '__main__':
    ID = os.getenv('ROS_HOSTNAME')
    ID = ID.split(".")[-1] if ID else ""

    try:
        gmc = GlobalMotorControl(ID)
        gmc.listener()

    except rospy.ROSInterruptException as e:
        print(e)

