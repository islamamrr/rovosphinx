#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

vl_vel = 0.0

def vl_joy_callback(input):
    global vl_vel
    
    vl_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('vl_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            vl_joy_sub = rospy.Subscriber("/MotorVL/state", Float64, vl_joy_callback)

            if vl_vel > 0:    
                subboob = "c" + str(funcs.map_val(vl_vel, 0, 1, 100, 400))
            elif vl_vel < 0:    
                subboob = "c" + str(funcs.map_val(vl_vel, 0, 1, 100, 400) * -1)
            else:
                subboob = "c" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")