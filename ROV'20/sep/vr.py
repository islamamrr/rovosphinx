#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

vr_vel = 0.0

def vr_joy_callback(input):
    global vr_vel
    
    vr_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('vr_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            vr_joy_sub = rospy.Subscriber("/MotorVR/state", Float64, vr_joy_callback)

            if vr_vel > 0:    
                subboob = "b" + str(funcs.map_val(vr_vel, 0, 1, 100, 400))
            elif vr_vel < 0:    
                subboob = "b" + str(funcs.map_val(vr_vel, 0, 1, 100, 400) * -1)
            else:
                subboob = "b" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")