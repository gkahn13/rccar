#!/usr/bin/env python

import rospy

from rccar.camera import Camera

if __name__ == '__main__':
    rospy.init_node('run_camera', anonymous=True)
    cam = Camera()
    cam.run()
