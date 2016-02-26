#!/usr/bin/python

# Imports
import rospy
import RPi.GPIO as GPIO
import trajectory
from std_msgs.msg import Float32, Int32, String

# Variables
node_name = 'motor_control_x'


class motor_control_x():

    def __init__(self):

        # GPIO pins
        self.enable_pin = 10
        self.control_pin = 8
        self.dir_pin = 12

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.control_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        # ROS init
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber("X_Pos", Float32, self.callback_pos)
        self.subscriber = rospy.Subscriber("X_Vel", Float32, self.callback_vel)
        self.subscriber = rospy.Subscriber("Motor_Kill", String, self.callback_kill)

        # ROS publishments
        self.pub_fb_x = rospy.Publisher('FB_Pulse_X', Int32, queue_size=10)

        # Constants
        self.dist_step = 0.127
        self.mode_step = 0.25  # 4 pulses per step
        self.pulse = self.dist_step * self.mode_step

        # Movement mode
        self.vel_flag = 0

        # Position control
        self.delta = 0
        self.direction = 0
        self.f_max = 3500
        self.f_min = 500
        self.plat_size = 0.5
        self.max_slope = 10
        self.nb_pulse = 0
        self.error_pulse = 0  # Rounding error
        self.pulse_counter = 0

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
        self.pub_fb_x.publish(self.pulse_counter)

    # Callback for new velocity
    def callback_vel(self, data):
        self.vel_flag = 1
        self.vel = data.data

    # Callback to kill motor
    def callback_kill(self, data):
        if data.data == node_name:
            rospy.signal_shutdown(node_name)

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
        mcx = motor_control_x()
        mcx.listener()

    except rospy.ROSInterruptException as e:
        print(e)

