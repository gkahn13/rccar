#!/usr/bin/env python

import rospy

from rccar.pid import PID

if __name__ == '__main__':
    rospy.init_node('run_pid', anonymous=True)
    ard = PID()
    ard.run()

