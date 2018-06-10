import numpy as np
import cv2

import rospy
import sensor_msgs.msg

class Camera:

    def __init__(self, rate=30.):
        self._rate = rate

        self._cam_pub = rospy.Publisher('camera/image_raw/compressed', sensor_msgs.msg.CompressedImage, queue_size=100)
        
        self._cam = None
        for cam_num in [0, 1]:
            cam = cv2.VideoCapture(cam_num)
            if cam.read()[0]:
                self._cam = cam
                self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self._cam.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
                self._cam.set(cv2.CAP_PROP_CONTRAST, 0.2)
                self._cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                self._cam.set(cv2.CAP_PROP_EXPOSURE, 0.08)
                break

        assert(self._cam is not None)

    def get_image(self):
        ret, data = self._cam.read()
        if ret:
            return data # bgr

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            img_np = self.get_image()
            msg = sensor_msgs.msg.CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img_np)[1]).tostring()
            self._cam_pub.publish(msg)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('run_camera', anonymous=True)
    cam = Camera()
    cam.run()
