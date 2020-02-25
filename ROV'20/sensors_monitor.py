#!/usr/bin/env python
import rospy
from ros_essentials_cpp.msg import Sensors

def sensors_callback(input):
    #get_caller_id(): Get fully resolved name of local node
    sns1 = input.isco
    rospy.loginfo(sns1)

def read_send():

    rospy.init_node('sensors_monitor', anonymous=True)
    velocity_publisher = rospy.Publisher("/", Sensors, queue_size=10)
    
    rospy.Subscriber("sensors", Sensors, sensors_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    read_send()
