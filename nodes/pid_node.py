#!/usr/bin/env python

import rospy

from rccar.car import Car
from rccar.arduino import Arduino

if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    rospy.sleep(3.0)
    
    car = Car()
    car.calibrate()
    prev_state = car.get_state()
    
    rate = rospy.Rate(100.)
    while not rospy.is_shutdown():
        rate.sleep()
        
        curr_state = car.get_state()
        
        if (curr_state == Arduino.STATE_CAFFE_CAFFE_STEER_HUMAN_MOTOR or \
            curr_state == Arduino.STATE_CAFFE_HUMAN_STEER_HUMAN_MOTOR or \
            curr_state == Arduino.STATE_CAFFE_CAFFE_STEER_CAFFE_MOTOR or \
            curr_state == Arduino.STATE_CAFFE_HUMAN_STEER_CAFFE_MOTOR):
            if prev_state == Arduino.STATE_HUMAN_FULL_CONTROL:
                # set desired velocity to current velocity (so last known human velocity)
                des_vel = car.get_velocity()
                if des_vel > 1.0:
                    print('Setting velocity to {0}'.format(des_vel))
                    car.set_velocity(des_vel) 
                else:
                    print('Desired velocity {0} too slow for PID!'.format(des_vel))
                    car.reset()
        else:
            if prev_state != curr_state:
                print('Turning off PID')
                car.reset()
            
        if (prev_state == Arduino.STATE_LOCK_CALIBRATE) and (prev_state != curr_state):
            print('Calibrating')
            car.calibrate()
            car.reset()
            
        prev_state = curr_state
    
    
