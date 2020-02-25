#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

s2s_vel = 0.0

def s2s_joy_callback(input):
    global s2s_vel
    
    s2s_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('s2s_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            s2s_joy_sub = rospy.Subscriber("/MotorS2S/state", Float64, s2s_joy_callback)

            if s2s_vel > 0:    
                subboob = "f" + str(funcs.map_val(s2s_vel, 0, 1, 100, 400))
            elif s2s_vel < 0:    
                subboob = "f" + str(funcs.map_val(s2s_vel, 0, -1, 100, 400) * -1)
            else:
                subboob = "f" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")