import rospy

from rccar.car import Car

if __name__ == '__main__':
    rospy.init_node('test_car', anonymous=True)
    car = Car()
    car.calibrate()
    
    import IPython; IPython.embed()
    
