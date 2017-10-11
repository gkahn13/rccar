import os, serial, threading, queue, collections
import threading

import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

class Arduino:


    def __init__(self, baudrate=115200, timeout=0.25, write_timeout=0):
        ### setup serial ports
        self.ser = self._setup_serial(baudrate, timeout, write_timeout)
        assert(self.ser is not None)
        
        ### control publishers (from Arduino)
        self.mode_pub = rospy.Publisher('mode', std_msgs.msg.Int32, queue_size=100)
        self.steer_pub = rospy.Publisher('steer', std_msgs.msg.Float32, queue_size=100)
        self.motor_pub = rospy.Publisher('motor', std_msgs.msg.Float32, queue_size=100)
        ### subscribers (info sent to Arduino)
        self.cmd_steer_sub = rospy.Subscriber('cmd/steer', std_msgs.msg.Float32,
                                              callback=self._cmd_steer_callback)
        self.cmd_motor_sub = rospy.Subscriber('cmd/motor', std_msgs.msg.Float32,
                                              callback=self._cmd_motor_callback)
        self.cmd_steer_queue = queue.Queue()
        self.cmd_motor_queue = queue.Queue()

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

    def _serial_thread(self):
        """
        Sends/receives message from servos serial and
        publishes/subscribes to ros
        """
        info = dict()
        
        while not rospy.is_shutdown():
            # ser_str = self.ser.readline()
            # if len(ser_str) > 0:
            #     print(ser_str)
            # info['steer'] = 0
            # info['motor'] = 0

            ### read serial
            try:
                ser_str = self.ser.readline()
                ser_tuple = eval(ser_str)
                assert(len(ser_tuple) == 3)
            except:
                continue
            ### parse servos serial
            info['mode'], info['steer'], info['motor'] = ser_tuple
            ### publish ROS
            self.mode_pub.publish(std_msgs.msg.Int32(info['mode']))
            self.steer_pub.publish(std_msgs.msg.Float32(info['steer']))
            self.motor_pub.publish(std_msgs.msg.Float32(info['motor']))

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
                # print('write_str: {0}'.format(write_str))
                self.ser.write(write_str)

                # write_int = int(1e4 * steer_int) + motor_int
                # write_bytes = write_int.to_bytes(length=4, byteorder='little')
                # self.ser.reset_output_buffer()
                # self.ser.write(write_bytes)

            ### print stuff
            print('({0}, {1:6.2f}, {2:6.2f})'.format(info['mode'], info['steer'], info['motor']))

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
