#!/usr/bin/python

# Imports
import ast
import pigpio
import rospy
from std_msgs.msg import String
import trajectory_xy
from trajectory_xy import X, Y

class motor_control_xy():
    """Multiple attributes are lists of two elements --> [X, Y]"""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        # Expected format of Pulse_XY is '[int_x, int_y]'
        self.subscriber = rospy.Subscriber('Pulse_XY', String, self.callback_pos)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error_XY', String, queue_size=10)

        # Frequency trapeze constants
        self.f_max = [10000, 14000]
        self.f_min = [500, 500]
        self.max_slope = [5, 5]

        # Position control
        self.single_axis_mode = 0
        self.delta = [0, 0]
        self.direction = [0, 0]
        self.nb_pulse = [0, 0]

        # GPIO pins
        self.enable_pin = [15, 20]  # pins 10 and 38
        self.clock_pin = [14, 21]   # pins 8 and 40
        self.dir_pin = [18, 16]     # pins 12 and 36

        # Init GPIO
        self.gpio = pigpio.pi()
        self.gpio.set_mode(self.enable_pin[X], pigpio.OUTPUT)
        self.gpio.set_mode(self.enable_pin[Y], pigpio.OUTPUT)
        self.gpio.set_mode(self.clock_pin[X], pigpio.OUTPUT)
        self.gpio.set_mode(self.clock_pin[Y], pigpio.OUTPUT)
        self.gpio.set_mode(self.dir_pin[X], pigpio.OUTPUT)
        self.gpio.set_mode(self.dir_pin[Y], pigpio.OUTPUT)

        # GPIO output init
        self.gpio.write(self.enable_pin[X], pigpio.HIGH)
        self.gpio.write(self.enable_pin[Y], pigpio.HIGH)
        self.gpio.write(self.clock_pin[X], pigpio.LOW)
        self.gpio.write(self.clock_pin[Y], pigpio.LOW)
        self.gpio.write(self.dir_pin[X], self.direction[X])
        self.gpio.write(self.dir_pin[Y], self.direction[Y])

    # Callback for new position
    def callback_pos(self, data):
        try:
            self.delta = ast.literal_eval(data.data)
            assert len(self.delta) == 2
            assert type(self.delta[X]) == int
            assert type(self.delta[Y]) == int
            assert self.delta[X] < 65536
            assert self.delta[Y] < 65536

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for X, Y: {0}'.format(e)
            print(msg)
            self.error.publish(msg)
            return

        trajectory_xy.pos_move(self)
        self.done_move.publish(self.node_name)

    # Callback to kill motor
    # Do we keep this? Perhaps only set enable GPIO pins to HIGH...
    # def callback_kill(self, data):
    #     if data.data == self.node_name:
    #         rospy.signal_shutdown(self.node_name)

    # Listening function
    def listener(self):
        rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        mcxy = motor_control_xy()
        mcxy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

