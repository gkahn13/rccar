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
    RECV_FLOATS = 18
    STOP_CHAR = b'>'

    MAX_SERIAL_EXCEPTIONS = 20

    def __init__(self, baudrate=115200, timeout=0.25, write_timeout=0):
        ### setup serial ports
        self.ser = self._setup_serial(baudrate, timeout, write_timeout)
        assert(self.ser is not None)
        
        ### control publishers (from Arduino)
        self.mode_pub = rospy.Publisher('mode', std_msgs.msg.Int32, queue_size=10)
        self.steer_pub = rospy.Publisher('steer', std_msgs.msg.Float32, queue_size=10)
        self.motor_pub = rospy.Publisher('motor', std_msgs.msg.Float32, queue_size=10)
        self.battery_a_pub = rospy.Publisher('battery/a', std_msgs.msg.Float32, queue_size=10)
        self.battery_b_pub = rospy.Publisher('battery/b', std_msgs.msg.Float32, queue_size=10)
        self.enc_left_pub = rospy.Publisher('encoder/left', std_msgs.msg.Float32, queue_size=10)
        self.enc_right_pub = rospy.Publisher('encoder/right', std_msgs.msg.Float32, queue_size=10)
        self.ori_pub = rospy.Publisher('orientation/quat', geometry_msgs.msg.Quaternion, queue_size=10)
        self.imu_pub = rospy.Publisher('imu', geometry_msgs.msg.Accel, queue_size=10)
        self.bumper_pub = rospy.Publisher('bumper', std_msgs.msg.Int32, queue_size=10)
        ### secondary publishers
        self.enc_pub = rospy.Publisher('encoder/both', std_msgs.msg.Float32, queue_size=10)
        self.rpy_pub = rospy.Publisher('orientation/rpy', geometry_msgs.msg.Vector3, queue_size=10)
        self.battery_low_pub = rospy.Publisher('battery/low', std_msgs.msg.Int32, queue_size=10)
        self.collision_pub = rospy.Publisher('collision/all', std_msgs.msg.Int32, queue_size=10)
        self.collision_flip_pub = rospy.Publisher('collision/flip', std_msgs.msg.Int32, queue_size=10)
        self.collision_jolt_pub = rospy.Publisher('collision/jolt', std_msgs.msg.Int32, queue_size=10)
        self.collision_stuck_pub = rospy.Publisher('collision/stuck', std_msgs.msg.Int32, queue_size=10)
        self.collision_stuck_encoder_deque = collections.deque([], 40) # TODO: make larger to be more conservative
        self.collision_stuck_motor_deque = collections.deque([], 40)
        self.collision_stuck_end_idx = 20
        self.collision_stuck_start_idx = 0
        self.collision_bumper_zero = 1024
        self.collision_bumper_pub = rospy.Publisher('collision/bumper', std_msgs.msg.Int32, queue_size=10)
        ### subscribers (info sent to Arduino)
        self.cmd_steer_sub = rospy.Subscriber('cmd/steer', std_msgs.msg.Float32,
                                              callback=self._cmd_steer_callback)
        self.cmd_motor_sub = rospy.Subscriber('cmd/motor', std_msgs.msg.Float32,
                                              callback=self._cmd_motor_callback)
        self.cmd_steer_queue = Queue()
        self.cmd_motor_queue = Queue()
        self.collision_lock = threading.RLock()
        self.collision_occurred = False
        self.collision_sub = rospy.Subscriber('collision/all', std_msgs.msg.Int32, callback=self._collision_callback)
        
        self.num_serial_exceptions = 0
        
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
        try:
            if self.ser.read() != Arduino.START_CHAR:
                return False

            read_bytes = self.ser.read(4 * Arduino.RECV_FLOATS)
            stop_char = self.ser.read()

            if stop_char != Arduino.STOP_CHAR:
                return False
        except serial.serialutil.SerialException as e:
            self.num_serial_exceptions += 1
            if self.num_serial_exceptions > Arduino.MAX_SERIAL_EXCEPTIONS:
                raise e
            return None

        self.num_serial_exceptions = 0

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
        info['bumper'] = int(unpacked[17])
        return True

    def _publish_serial(self, info):
        ### raw topics
        self.mode_pub.publish(std_msgs.msg.Int32(info['mode']))
        self.steer_pub.publish(std_msgs.msg.Float32(info['steer']))
        self.motor_pub.publish(std_msgs.msg.Float32(info['motor']))
        self.battery_a_pub.publish(std_msgs.msg.Float32(info['batt_a']))
        self.battery_b_pub.publish(std_msgs.msg.Float32(info['batt_b']))
        self.enc_left_pub.publish(std_msgs.msg.Float32(np.sign(info['motor']) * info['enc_left']))
        self.enc_right_pub.publish(std_msgs.msg.Float32(np.sign(info['motor']) * info['enc_right']))
        self.ori_pub.publish(geometry_msgs.msg.Quaternion(*info['orientation']))
        self.imu_pub.publish(geometry_msgs.msg.Accel(linear=geometry_msgs.msg.Vector3(*info['acc']),
                                                     angular=geometry_msgs.msg.Vector3(*info['gyro'])))
        self.bumper_pub.publish(std_msgs.msg.Int32(info['bumper']))
        
        ### transformations of raw topics
        self.enc_pub.publish(std_msgs.msg.Float32(0.5 * np.sign(info['motor']) * (info['enc_left'] + info['enc_right'])))
        self.rpy_pub.publish(geometry_msgs.msg.Vector3(*tft.euler_from_quaternion(list(info['orientation'][1:]) + \
                                                                                    [info['orientation'][0]])))
        self.battery_low_pub.publish(std_msgs.msg.Int32(int((info['batt_a'] < 3.4 * 3) or (info['batt_b'] < 3.4 * 3))))

        coll_flip = (info['acc'][2] > 5.0)
        coll_jolt = (info['acc'][0] < -10.0) or (info['acc'][0] > 10.0)
        self.collision_stuck_encoder_deque.append(abs(0.5 * (info['enc_left'] + info['enc_right'])))
        self.collision_stuck_motor_deque.append(abs(info['motor']))
        if len(self.collision_stuck_encoder_deque) == self.collision_stuck_encoder_deque.maxlen:
            # if encoder is 0 for a long time and trying to motor
            coll_stuck = (np.median(list(self.collision_stuck_motor_deque)[:-self.collision_stuck_end_idx]) > 0.15) and (max(list(self.collision_stuck_encoder_deque)[self.collision_stuck_start_idx:]) < 1e-3)
        else:
            coll_stuck = False
        coll_bumper = (info['bumper'] >= self.collision_bumper_zero + 30)
        if (abs(info['motor']) < 0.05) and (0.5 * (info['enc_left'] + info['enc_right']) < 1e-4):
            self.collision_bumper_zero = info['bumper']
        self.collision_pub.publish(std_msgs.msg.Int32(int(coll_flip or coll_jolt or coll_stuck or coll_bumper)))
        self.collision_flip_pub.publish(std_msgs.msg.Int32(int(coll_flip)))
        self.collision_jolt_pub.publish(std_msgs.msg.Int32(int(coll_jolt)))
        self.collision_stuck_pub.publish(std_msgs.msg.Int32(int(coll_stuck)))
        self.collision_bumper_pub.publish(std_msgs.msg.Int32(int(coll_bumper)))
        
    def run(self):
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

            with self.collision_lock:
                if self.collision_occurred:
                    write_info['steer'] = 0.
                    write_info['motor'] = 0.
                self.collision_occurred = False
                    
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

    def _collision_callback(self, msg):
        if msg.data == 1:
            with self.collision_lock:
                self.collision_occurred = True
            
if __name__ == '__main__':
    rospy.init_node('run_arduino', anonymous=True)
    ard = Arduino()
    ard.run()
    
