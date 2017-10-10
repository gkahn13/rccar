#!/usr/bin/env python

import os, sys, shutil, subprocess, time

import rospy

if __name__ == '__main__':
    rospy.init_node('rosbag_node', anonymous=True)

    assert(len(sys.argv) >= 3)

    bag_rec_folder = sys.argv[1] # '/home/ubuntu/catkin_ws/src/rccar/rosbags'
    bag_mv_folder = sys.argv[2] # '/media/ubuntu/3131-3031/rosbags'

    ### create dedicated experiment folder
    exp_folder_nums = [int(d.replace('exp', ''))
                       for d in os.listdir(bag_mv_folder)
                       if os.path.isdir(os.path.join(bag_mv_folder, d)) and ('exp' in d)]
    exp_num = max(exp_folder_nums) + 1 if len(exp_folder_nums) > 0 else 0
    
    bag_mv_folder = os.path.join(bag_mv_folder, 'exp{0}'.format(exp_num))

    assert(os.path.exists(bag_rec_folder))
    assert(not os.path.exists(bag_mv_folder))

    os.makedirs(bag_mv_folder)

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        for f in os.listdir(bag_rec_folder):
            if '.bag' != os.path.splitext(f)[1]:
                continue
        
            print('Moving {0}'.format(f))
            f_rec = os.path.join(bag_rec_folder, f)
            f_mv = os.path.join(bag_mv_folder, f)
            # shutil.copy(f_rec, f_mv)
            start = time.time()
            subprocess.call(['mv', f_rec, f_mv])
            elapsed = time.time() - start
            print('Done in {0} secs\n'.format(elapsed))
            
        rate.sleep()

