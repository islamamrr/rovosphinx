#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ros_essentials_cpp.msg import Thrust_vel
from std_msgs.msg import String
import funcs

fr_vel = fl_vel = s2s_vel = vr_vel = vl_vel = vc_vel = 0

def all_callback(input):
    global fr_vel
    global fl_vel
    global vr_vel
    global vl_vel
    global vc_vel
    global s2s_vel

    if input.data[0] == 'a':
        vc_vel = float(input.data[1:])
    elif input.data[0] == 'b':
        vr_vel = float(input.data[1:])        
    elif input.data[0] == 'c':
        vl_vel = float(input.data[1:])        
    elif input.data[0] == 'd':
        fr_vel = float(input.data[1:])        
    elif input.data[0] == 'e':
        fl_vel = float(input.data[1:])        
    elif input.data[0] == 'f':
        s2s_vel = float(input.data[1:])        
        



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():

            rospy.init_node('thrust_ctrl', anonymous=True)

            velocity_publisher = rospy.Publisher("/thrusters_vels", Thrust_vel, queue_size=10)

            all_sub = rospy.Subscriber("/collection", String, all_callback)

            all_vels = Thrust_vel()            
            
            all_vels.vc = vc_vel
            all_vels.vr = vr_vel
            all_vels.vl = vl_vel
            all_vels.fr = fr_vel
            all_vels.fl = fl_vel
            all_vels.s2s = s2s_vel

            loop_rate = rospy.Rate(50)
            velocity_publisher.publish(all_vels)

            loop_rate.sleep()

            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")