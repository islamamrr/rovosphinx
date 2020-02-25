#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import funcs

fl_vel = 0.0

def fl_joy_callback(input):
    global fl_vel
    
    fl_vel = round(input.data, 2)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('fl_node', anonymous=True)

            velocity_publisher = rospy.Publisher("/collection", String, queue_size=10)

            fl_joy_sub = rospy.Subscriber("/MotorHL/state", Float64, fl_joy_callback)

            if fl_vel > 0:    
                subboob = "e" + str(funcs.map_val(fl_vel, 0, 1, 100, 400))
            elif fl_vel < 0:    
                subboob = "e" + str(funcs.map_val(fl_vel, 0, 1, 100, 400) * -1)
            else:
                subboob = "e" + str(0)

            loop_rate = rospy.Rate(10)
            velocity_publisher.publish(subboob)

            loop_rate.sleep()            


            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")