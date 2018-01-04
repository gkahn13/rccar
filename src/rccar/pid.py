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
        self._collision_stamp = None
        self._collision_sub = rospy.Subscriber('collision/all', std_msgs.msg.Int32, callback=self._collision_callback)
        self._cmd_vel_stamp = rospy.Time.now()
        self._cmd_vel_sub = rospy.Subscriber('cmd/vel', std_msgs.msg.Float32, callback=self._cmd_vel_callback)
        self._encoder_sub = rospy.Subscriber('encoder/both', std_msgs.msg.Float32, callback=self._encoder_callback)
        self._motor_sub = rospy.Subscriber('motor', std_msgs.msg.Float32, callback=self._motor_callback)
        self._mode_sub = rospy.Subscriber('mode', std_msgs.msg.Int32, callback=self._mode_callback)
        self._is_enabled = True
        self._enable_sub = rospy.Subscriber('pid/enable', std_msgs.msg.Empty, callback=self._enable_callback)
        self._disable_sub = rospy.Subscriber('pid/disable', std_msgs.msg.Empty, callback=self._disable_callback)
        ### publishers
        self._motor_pub = rospy.Publisher('cmd/motor', std_msgs.msg.Float32, queue_size=100)
    
    ###########
    ### ROS ###
    ###########

    def _collision_callback(self, msg):
        if msg.data == 1:
            self._collision_stamp = rospy.Time.now()
    
    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg.data
        self._cmd_vel_stamp = rospy.Time.now()
    
    def _encoder_callback(self, msg):
        self._encoder = msg.data
    
    def _motor_callback(self, msg):
        self._motor = msg.data

    def _mode_callback(self, msg):
        if msg.data != 2:
            self._cmd_vel = 0.

    def _enable_callback(self, msg):
        self._is_enabled = True

    def _disable_callback(self, msg):
        self._is_enabled = False
            
    ################
    ### PID loop ###
    ################

    def _update_rosparams(self):
        self._kp = rospy.get_param('~kp')
        self._kd = rospy.get_param('~kd')

    def run(self):
        err = 0.
        err_last = 0.

        rate = rospy.Rate(self._hz)

        while not rospy.is_shutdown():
            self._update_rosparams()

            curr_vel = self._encoder
            err = self._cmd_vel - curr_vel
            new_motor = self._motor + self._kp * err - self._kd * (err - err_last) * self._dt
            new_motor = np.clip(new_motor, -1., 1.)

            if self._collision_stamp is not None and  self._collision_stamp >= self._cmd_vel_stamp:
                new_motor = 0.

            if self._is_enabled:
                self._motor_pub.publish(std_msgs.msg.Float32(new_motor))
    
            err_last = err

            rate.sleep()

