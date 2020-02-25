#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

vc_vel = 0.0

def vc_joy_callback(input):
    global vc_vel
    
    vc_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('vc_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            vc_joy_sub = rospy.Subscriber("/MotorVC/state", Float64, vc_joy_callback)

            if vc_vel > 0:    
                subboob = "a" + str(funcs.map_val(vc_vel, 0, 1, 100, 400))
            elif vc_vel < 0:    
                subboob = "a" + str(funcs.map_val(vc_vel, 0, 1, 100, 400) * -1)
            else:
                subboob = "a" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")