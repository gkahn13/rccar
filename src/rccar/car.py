import threading

import numpy as np

import rospy
import std_msgs.msg

class Car:

    def __init__(self):
        ### subscribers
        self.state = None
        self.steer = None
        self.motor = None
        self.encoder = None
        self.state_sub = rospy.Subscriber('state', std_msgs.msg.Int32,
                                          callback=self._save_callback,
                                          callback_args=('state',))
        self.steer_sub = rospy.Subscriber('steer', std_msgs.msg.Float32,
                                          callback=self._save_callback,
                                          callback_args=('steer',))
        self.motor_sub = rospy.Subscriber('motor', std_msgs.msg.Float32,
                                          callback=self._save_callback,
                                          callback_args=('motor',))
        self.encoder_sub = rospy.Subscriber('encoder', std_msgs.msg.Float32,
                                            callback=self._save_callback,
                                            callback_args=('encoder',))

        ### publishers
        self.steer_cmd_pub = rospy.Publisher('cmd/steer', std_msgs.msg.Float32, queue_size=100)
        self.motor_cmd_pub = rospy.Publisher('cmd/motor', std_msgs.msg.Float32, queue_size=100)

        ### calibration
        self.zero_motor = None
        self.zero_steer = None

        ### vel PID
        self.is_vel_control = False
        self.des_vel = 0.
        threading.Thread(target=self._vel_control_thread).start()

    #################
    ### Resetters ###
    #################

    def calibrate(self):
        """ Assumes car is calibrated and not moving """
        self.steer = None
        self.motor = None

        print('\nCalibrating...')
        rate = rospy.Rate(10.)
        while not rospy.is_shutdown() and (self.steer is None) and (self.motor is None):
            rate.sleep()

        self.zero_motor = self.motor
        self.zero_steer = self.steer

        print('Calibrating complete!\n')

    ###############
    ### Getters ###
    ###############
    
    def get_state(self):
        return self.state
        
    def get_steer(self):
        return self.steer
        
    def get_motor(self):
        return self.motor

    def get_velocity(self):
        assert(self.zero_motor is not None) # make sure to calibrate first!
        return np.sign(self.get_motor() - self.zero_motor) * np.abs(self.encoder)

    def is_velocity_control(self):
        return self.is_vel_control
        
    ###############
    ### Setters ###
    ###############

    def reset(self):
        # requires calibrate
        self.turn_off_velocity_control()
        assert(self.zero_motor is not None) 
        assert(self.zero_steer is not None)
        self.set_motor(self.zero_motor)
        self.set_steer(self.zero_steer)
    
    def set_steer(self, steer):
        assert(0 <= steer and steer <= 100)
        self.steer_cmd_pub.publish(std_msgs.msg.Float32(steer))
        
    def set_motor(self, motor):
        assert(0 <= motor and motor <= 100)
        self.turn_off_velocity_control()
        self.motor_cmd_pub.publish(std_msgs.msg.Float32(motor))

    def set_velocity(self, des_vel):
        self.des_vel = des_vel
        self.is_vel_control = True

    def turn_off_velocity_control(self):
        self.is_vel_control = False
        self.des_vel = 0.

    #################
    ### Callbacks ###
    #################

    def _save_callback(self, msg, args):
        attr_name = args[0]
        setattr(self, attr_name, msg.data)

    ###############
    ### Threads ###
    ###############

    def _vel_control_thread(self):
        dt = 1./20.
        rate = rospy.Rate(1./dt)
        Kp = 0.3
        Kd = 0.3

        err_last = 0.
        err = 0.

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.is_vel_control:
                continue

            curr_motor = self.get_motor()
            err = self.des_vel - self.get_velocity()
            new_motor = curr_motor + Kp * err - Kd * (err - err_last) * dt
            
            # update for Kd
            err_last = err

            # clip            
            new_motor = np.clip(new_motor, 0, 100)
            # new_motor = np.clip(new_motor, curr_motor - 0.5, curr_motor + 8.0)
            
#            print('curr_vel: {0}'.format(self.get_velocity()))
#            print('des_vel: {0}'.format(self.des_vel))
#            print('err: {0}'.format(err))
#            print('curr_motor: {0}'.format(self.get_motor()))
#            print('new_motor: {0}'.format(new_motor))
#            print('')
            self.motor_cmd_pub.publish(std_msgs.msg.Float32(new_motor))
            

            




