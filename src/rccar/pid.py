import numpy as np

import rospy

import std_msgs.msg

class PID:

    def __init__(self):
        self._kp = 0.0
        self._kd = 0.0

        self._hz = 20.
        self._dt = 1. / self._hz

        self._cmd_vel = 0.
        self._encoder = 0.
        self._motor = 0.
    
        ### subscribers
        self._cmd_vel_sub = rospy.Subscriber('cmd/vel', std_msgs.msg.Float32, callback=self._cmd_vel_callback)
        self._encoder_sub = rospy.Subscriber('encoder/both', std_msgs.msg.Float32, callback=self._encoder_callback)
        self._motor_sub = rospy.Subscriber('motor', std_msgs.msg.Float32, callback=self._motor_callback)
        ### publishers
        self._motor_pub = rospy.publisher('cmd/motor', std_msgs.msg.Float32, queue_size=100)
    
    ###########
    ### ROS ###
    ###########

    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg.data
    
    def _encoder_callback(self, msg):
        self._encoder = msg.data
    
    def _motor_callback(self, msg):
        self._motor = msg.data

    ################
    ### PID loop ###
    ################

    def _update_rosparams:
        self._kp = rospy.get_param('kp', 0.03)
        self._kd = rospy.get_param('kd', 0.0)
        

    def run(self):
        err = 0.
        err_last = 0.

        rate = rospy.Rate(self._hz)

        while not rospy.is_shutdown():
            self._update_rosparams()

            curr_vel = np.sign(self._motor) * self._encoder
            err = self._cmd_vel - curr_vel
            new_motor = self._motor + self._kp * err - self._kd * (err - err_last) * self._dt
            new_motor = np.clip(new_motor, -1., 1.)

            self._motor_pub.publish(std_msgs.msg.Float32(new_motor))
    
            err_last = err

            rate.sleep()

