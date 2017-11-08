import sys
if sys.version_info[0] == 2:
    from Queue import Queue
else:
    from queue import Queue
import os, serial, threading, collections
import threading
import struct

import numpy as np

import rospy
import tf.transformations as tft
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

class Arduino:
    START_CHAR = b'<'
    RECV_FLOATS = 17
    STOP_CHAR = b'>'

    def __init__(self, baudrate=115200, timeout=0.25, write_timeout=0):
        ### setup serial ports
        self.ser = self._setup_serial(baudrate, timeout, write_timeout)
        assert(self.ser is not None)
        
        ### control publishers (from Arduino)
        self.mode_pub = rospy.Publisher('mode', std_msgs.msg.Int32, queue_size=100)
        self.steer_pub = rospy.Publisher('steer', std_msgs.msg.Float32, queue_size=100)
        self.motor_pub = rospy.Publisher('motor', std_msgs.msg.Float32, queue_size=100)
        self.battery_a_pub = rospy.Publisher('battery/a', std_msgs.msg.Float32, queue_size=100)
        self.battery_b_pub = rospy.Publisher('battery/b', std_msgs.msg.Float32, queue_size=100)
        self.enc_left_pub = rospy.Publisher('encoder/left', std_msgs.msg.Float32, queue_size=100)
        self.enc_right_pub = rospy.Publisher('encoder/right', std_msgs.msg.Float32, queue_size=100)
        self.ori_pub = rospy.Publisher('orientation', geometry_msgs.msg.Quaternion, queue_size=100)
        self.imu_pub = rospy.Publisher('imu', geometry_msgs.msg.Accel, queue_size=100)
        ### secondary publishers
        self.enc_pub = rospy.Publisher('encoder', std_msgs.msg.Float32, queue_size=100)
        self.rpy_pub = rospy.Publisher('orientation/rpy', geometry_msgs.msg.Vector3, queue_size=100)
        self.battery_low_pub = rospy.Publisher('battery/low', std_msgs.msg.Bool, queue_size=100)
        self.collision_pub = rospy.Publisher('collision', std_msgs.msg.Bool, queue_size=100)
        self.collision_flip_pub = rospy.Publisher('collision/flip', std_msgs.msg.Bool, queue_size=100)
        self.collision_jolt_pub = rospy.Publisher('collision/jolt', std_msgs.msg.Bool, queue_size=100)
        self.collision_stuck_pub = rospy.Publisher('collision/stuck', std_msgs.msg.Bool, queue_size=100)
        ### subscribers (info sent to Arduino)
        self.cmd_steer_sub = rospy.Subscriber('cmd/steer', std_msgs.msg.Float32,
                                              callback=self._cmd_steer_callback)
        self.cmd_motor_sub = rospy.Subscriber('cmd/motor', std_msgs.msg.Float32,
                                              callback=self._cmd_motor_callback)
        self.cmd_steer_queue = Queue()
        self.cmd_motor_queue = Queue()

        ### start background ros thread
        print('Starting threads')
        #threading.Thread(target=self._serial_thread).start()
        self._serial_thread()

    #############
    ### Setup ###
    #############
    
    def _setup_serial(self, baudrate, timeout, write_timeout):
        sers = []
        ACM_ports = [os.path.join('/dev', p) for p in os.listdir('/dev') if 'ttyACM' in p]
        for ACM_port in ACM_ports:
            try:
                sers.append(serial.Serial(ACM_port, baudrate=baudrate, timeout=timeout, write_timeout=write_timeout))
                print('Opened {0}'.format(ACM_port))
            except:
                pass

        assert(len(sers) == 1)
        ser = sers[0]
        return ser

    ###################
    ### ROS methods ###
    ###################

    def _read_serial(self, info):
        if self.ser.read() != Arduino.START_CHAR:
            return False

        read_bytes = self.ser.read(4 * Arduino.RECV_FLOATS)
        stop_char = self.ser.read()

        if stop_char != Arduino.STOP_CHAR:
            return False

        try:
            unpacked = struct.unpack('<{0:d}f'.format(Arduino.RECV_FLOATS), read_bytes)
        except:
            return False

        info['mode'] = int(unpacked[0])
        info['steer'] = unpacked[1]
        info['motor'] = unpacked[2]
        info['batt_a'] = unpacked[3]
        info['batt_b'] = unpacked[4]
        info['enc_left'] = unpacked[5]
        info['enc_right'] = unpacked[6]
        info['orientation'] = unpacked[7:11]
        info['acc'] = unpacked[11:14]
        info['gyro'] = unpacked[14:17]

        return True

    def _publish_serial(self, info):
        ### raw topics
        self.mode_pub.publish(std_msgs.msg.Int32(info['mode']))
        self.steer_pub.publish(std_msgs.msg.Float32(info['steer']))
        self.motor_pub.publish(std_msgs.msg.Float32(info['motor']))
        self.battery_a_pub.publish(std_msgs.msg.Float32(info['batt_a']))
        self.battery_b_pub.publish(std_msgs.msg.Float32(info['batt_b']))
        self.enc_left_pub.publish(std_msgs.msg.Float32(info['enc_left']))
        self.enc_right_pub.publish(std_msgs.msg.Float32(info['enc_right']))
        self.ori_pub.publish(geometry_msgs.msg.Quaternion(*info['orientation']))
        self.imu_pub.publish(geometry_msgs.msg.Accel(linear=geometry_msgs.msg.Vector3(*info['acc']),
                                                     angular=geometry_msgs.msg.Vector3(*info['gyro'])))

        ### transformations of raw topics
        self.enc_pub.publish(std_msgs.msg.Float32(0.5 * (info['enc_left'] + info['enc_right'])))
        self.rpy_pub.publish(geometry_msgs.msg.Vector3(*tft.euler_from_quaternion(list(info['orientation'][1:]) + \
                                                                                    [info['orientation'][0]])))
        self.battery_low_pub.publish(std_msgs.msg.Bool((info['batt_a'] < 3.4 * 3) or (info['batt_b'] < 3.4 * 3)))

        coll_flip = (info['acc'][2] < 5.0)
        coll_jolt = (abs(info['acc'][0]) > 10.0)
        coll_stuck = (info['motor'] > 0.2 and (abs(0.5 * (info['enc_left'] + info['enc_right'])) < 0.2))
        self.collision_pub.publish(std_msgs.msg.Bool(coll_flip or coll_jolt or coll_stuck))
        self.collision_flip_pub.publish(std_msgs.msg.Bool(coll_flip))
        self.collision_jolt_pub.publish(std_msgs.msg.Bool(coll_jolt))
        self.collision_stuck_pub.publish(std_msgs.msg.Bool(coll_stuck))

    def _serial_thread(self):
        """
        Sends/receives message from servos serial and
        publishes/subscribes to ros
        """
        info = dict()
        
        while not rospy.is_shutdown():
            if not self._read_serial(info):
                continue

            ### publish ROS
            self._publish_serial(info)

            ### write serial
            write_info = collections.defaultdict(int)
            write_to_ser = False
            for var, queue in (('steer', self.cmd_steer_queue),
                               ('motor', self.cmd_motor_queue)):
                if not queue.empty():
                    write_to_ser = True
                    write_info[var] = queue.get()
                    # print('Setting {0} to {1}'.format(var, write_info[var]))
                else:
                    write_info[var] = info[var]

            if write_to_ser:
                steer_int = int(np.clip(int(1e4 * 0.5 * (write_info['steer'] + 1)), 1, 9999))
                motor_int = int(np.clip(int(1e4 * 0.5 * (write_info['motor'] + 1)), 1, 9999))

                write_str = str.encode('({0:04d}{1:04d})'.format(steer_int, motor_int))
                self.ser.write(write_str)

    #################
    ### Callbacks ###
    #################
            
    def _cmd_steer_callback(self, msg):
        if msg.data >= -1. and msg.data <= 1.:
            self.cmd_steer_queue.put(msg.data)
        
    def _cmd_motor_callback(self, msg):
        if msg.data >= -1. and msg.data <= 1.:
            self.cmd_motor_queue.put(msg.data)

if __name__ == '__main__':
    rospy.init_node('run_arduino', anonymous=True)
    ard = Arduino()
    rospy.spin()
