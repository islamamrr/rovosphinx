#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

fr_vel = 0.0

def fr_joy_callback(input):
    global fr_vel
    
    fr_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('fr_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            fr_joy_sub = rospy.Subscriber("/MotorHR/state", Float64, fr_joy_callback)

            if fr_vel > 0:    
                subboob = "d" + str(funcs.map_val(fr_vel, 0, 1, 100, 400))
            elif fr_vel < 0:    
                subboob = "d" + str(funcs.map_val(fr_vel, 0, 1, 100, 400) * -1)
            else:
                subboob = "d" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")