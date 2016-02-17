#!/usr/bin/python

# Imports
import rospy
import RPi.GPIO as GPIO
import trajectory
from std_msgs.msg import Float32, String

# Variables
node_name = 'motor_control_y'


class motor_control_y():

    def __init__(self):

        # GPIO pins
        self.enable_pin = 38
        self.control_pin = 40
        self.dir_pin = 36

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.control_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        # ROS init
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber("Y_Pos", Float32, self.callback_pos)
        self.subscriber = rospy.Subscriber("Y_Vel", Float32, self.callback_vel)
        self.subscriber = rospy.Subscriber("Motor_Kill", String, self.callback_kill)

        # Constants
        self.dist_step = 0.127
        self.mode_step = 0.25  # 4 pulses per step

        # Movement mode
        self.vel_flag = 0

        # Position control
        self.delta = 0
        self.direction = 0
        self.f_max = 3500
        self.f_min = 500
        self.plat_size = 0.5
        self.pulse = self.dist_step * self.mode_step
        self.max_slope = 10
        self.error_pulse = 0  # Rounding error

        # Velocity frequency control
        self.vel = 0

        # GPIO output init
        GPIO.output(self.dir_pin, self.direction)
        GPIO.output(self.enable_pin, GPIO.HIGH)
        GPIO.output(self.control_pin, GPIO.LOW)


    # Callback for new position
    def callback_pos(self, data):
        self.delta = data.data
        trajectory.pos_move(self)

    # Callback for new velocity
    def callback_vel(self, data):
        self.vel_flag = 1
        self.vel = data.data

    # Callback to kill motor
    def callback_kill(self, data):
        if data.data == node_name:
            rospy.signal_shutdown(data.data)

    # Listening function
    def listener(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.vel_flag:
                trajectory.vel_move(self)
        # rospy.spin()  # old code - do we keep it?


# Main function
if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)

    try:
        mcy = motor_control_y()
        mcy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

