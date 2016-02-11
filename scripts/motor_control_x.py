#!/usr/bin/python

# Imports
import rospy
import RPi.GPIO as GPIO
import trajectory
from std_msgs.msg import Float32


class pos_control_x():

    def __init__(self):

        #init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(8, GPIO.OUT)
        GPIO.setup(10, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)

        #Constant
        self.dist_step = 0.127
        self.mode_step = 0.25 # 4 pulses per step

        #init var
        self.axis = 0
        self.delta = 0
        self.direction = 0
        self.f_max = 3500
        self.f_min = 500
        self.plat_size = 0.5
        self.pulse = self.dist_step * self.mode_step
        self.run = 1
        self.pos = 0
        self.max_slope = 10

        self.enable_pin = 10
        self.control_pin = 8
        self.dir_pin = 12

        self.error_pulse = 0 # Rounding error

        # GPIO output init
        GPIO.output(self.dir_pin, self.direction)
        GPIO.output(self.enable_pin, GPIO.HIGH)
        GPIO.output(self.control_pin, GPIO.LOW)

        # ROS init
        self.rate = rospy.Rate(10) # 10Hz
        self.subscriber = rospy.Subscriber("X_Move", Float32, self.callback)


    def callback(self, data):
        self.delta=data.data
        self.pos_move_x()

    def listener(self):
        rospy.spin()

    def pos_move_x(self):
        # Direction
        if self.delta < 0:
            self.direction = 0
            GPIO.output(self.dir_pin, self.direction)
        else:
            self.direction = 1
            GPIO.output(self.dir_pin, self.direction)

        # Movement
        nb_pulse_float = abs(self.delta)/self.pulse
        self.nb_pulse = int(round(nb_pulse_float))
        self.error_pulse += nb_pulse_float - self.nb_pulse
        # print("\nBefore error : {0}".format(self.error_pulse))
        # print("Before nb_pulse : {0}".format(self.nb_pulse))

        # Adjust values if error is greater than 1 pulse
        if self.error_pulse < -1:
            self.error_pulse += 1
            self.nb_pulse -= 1
        elif self.error_pulse > 1:
            self.error_pulse -= 1
            self.nb_pulse += 1

        # print("After error : {0}".format(self.error_pulse))
        # print("After nb_pulse : {0}".format(self.nb_pulse))

        if self.nb_pulse != 0:
            trajectory.axis_move(self.enable_pin, self.control_pin, self.nb_pulse,\
                            self.f_max, self.f_min, self.plat_size, self.max_slope)


if __name__ == '__main__':
    rospy.init_node('pos_control_x', anonymous=True)

    try:
        pc = pos_control_x()
        pc.listener()

    except rospy.ROSInterruptException as e:
        print(e)
        pass
