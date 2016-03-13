#!/usr/bin/python

# Imports
import pigpio
from platform_control.msg import IntList
import rospy
from std_msgs.msg import String
import trajectory
from trajectory import Z0, Z1, Z2

class MotorControlZ():
    """Multiple attributes are lists of three elements --> [Z0, Z1, Z2]"""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        # Expected format of Pulse_Z is [ID, int_z], where ID is 0, 1 or 2
        self.subscriber = rospy.Subscriber('Pulse_Z', IntList, self.callback_pos)
        self.subscriber = rospy.Subscriber('Motor_Kill', String, self.callback_kill)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

        # Frequency trapeze constants
        self.f_max = [6000, 6000, 6000]
        self.f_min = [500, 500, 500]
        self.max_slope = [5, 5, 5]

        # Position control
        self.mode = []  # Used for Z0, Z1 and Z2
        self.modes = [Z0, Z1, Z2]
        self.delta = [0, 0, 0]
        self.direction = [0, 0, 0]
        self.nb_pulse = [0, 0, 0]

        # GPIO pins
        self.enable_pin = [2, 2, 2]  # pins TODO
        self.clock_pin = [2, 2, 2]  # pin TODO
        self.dir_pin = [2, 2, 2]  # pin TODO
        # self.limit_sw = [0, 0, 0]  # pin TODO

        # Init GPIO
        self.gpio = pigpio.pi()
        for A in self.modes:
            self.gpio.set_mode(self.enable_pin[A], pigpio.OUTPUT)
        self.gpio.set_mode(self.clock_pin[0], pigpio.OUTPUT)
        self.gpio.set_mode(self.dir_pin[0], pigpio.OUTPUT)
        #self.gpio.set_mode(self.limit_sw[0], pigpio.INPUT)

        # GPIO output init
        for A in self.modes:
            self.gpio.write(self.enable_pin[A], pigpio.HIGH)
        self.gpio.write(self.clock_pin[0], pigpio.LOW)
        self.gpio.write(self.dir_pin[0], self.direction[0])

        # GPIO input init
        #self.gpio.set_glitch_filter(self.limit_sw[0], 50)
        #self.gpio.callback(self.limit_sw, pigpio.RISING_EDGE, \
        #                               self.callback_limit_sw_z)

    # Callback for new position
    def callback_pos(self, data):
        try:
            assert len(data.data) == 2
            mode, delta = data.data
            assert mode in self.modes
            assert type(delta) == int
            assert abs(delta) < 65536
            assert delta != 0
            self.mode = [mode]
            self.delta[mode] = delta

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for Z: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        trajectory.pos_move(self)
        self.done_move.publish(self.node_name)

    # def callback_limit_sw_z(self, gpio, level, tick):
    #     self.gpio.write(self.enable_pin[self.mode], pigpio.HIGH)
    #     print("Callback Limit Switch Z")

    def callback_kill(self, data):
        if data.data == self.node_name:
            print("Killing {0}".format(self.node_name))
            for A in self.modes:
                self.gpio.write(self.enable_pin[A], pigpio.HIGH)

    # Listening function
    def listener(self):
        rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        mcz = MotorControlZ()
        mcz.listener()

    except rospy.ROSInterruptException as e:
        print(e)

