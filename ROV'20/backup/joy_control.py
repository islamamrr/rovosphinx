#!/usr/bin/env python
import rospy
import funcs
from sensor_msgs.msg import Joy
from ros_essentials_cpp.msg import Sensor
from std_msgs.msg import Float64
from std_msgs.msg import Bool


L3_Y, L2, R3_X, R3_Y, R2 = 1, 2, 3, 4, 5
TRI, L1, R1 = 2, 4, 5
for_back_vel_r = 0.0
for_back_vel_l = 0.0
side_vel = 0.0
pitch_vel = 0.0
vertical_vel = 0.0
r2_vel_adj_flag = l2_vel_adj_flag = 0
depth_lock = depth_lock_check = depth_lock_check_flag = 0
current_depth = 0.0

def depth_state_cb(sensor_message):
    global current_depth

    current_depth = sensor_message.Depth

def joystick_callback(joy):

    # variables that need to maintain their values at each callback function call
    # or ones that are used in other functions are defined as global variables

    global for_back_vel_r
    global for_back_vel_l
    global side_vel
    global vertical_vel
    global pitch_vel
    
    global r2_vel_adj_flag
    global l2_vel_adj_flag
    
    global depth_lock
    global depth_lock_check
    global depth_lock_check_flag

    # function used to map vertical velocity and correct initial value error
    vertical_vel, r2_vel_adj_flag, l2_vel_adj_flag = funcs.get_vertical_vel(joy.axes[R2], joy.axes[L2], r2_vel_adj_flag, l2_vel_adj_flag)

    # holding the DL button won't cause the value to flicker
    if joy.buttons[TRI] == 1:
        if depth_lock_check_flag == 0:
            if depth_lock == 0:
                depth_lock = 1
                depth_setpoint_pub.publish(current_depth)
            else:
                depth_lock = 0
            depth_lock_check_flag = 1
    else:
        depth_lock_check_flag = 0
    
    # if joy.axes[R3_Y] == 0:
    #     pitch_vel = vertical_vel
    # else:
    pitch_vel = round(joy.axes[R3_Y], 1)

    if joy.buttons[R1] == 1:
        for_back_vel_r = 0.6
        for_back_vel_l = -0.6
    
    elif joy.buttons[L1] == 1:
        for_back_vel_r = -0.6
        for_back_vel_l = 0.6

    # if joy.buttons[L1] == 0 and joy.buttons[R1] == 0:
    else:
        for_back_vel_r = -round(joy.axes[L3_Y], 1)
        for_back_vel_l = -round(joy.axes[L3_Y], 1)
    
    print for_back_vel_l


    side_vel = round(joy.axes[R3_X], 1)
    ########## to compensate for any moment caused by the s2s thruster,
    ########## change the values of the forward thrusters while maintaining their opposite signs
    # if side_vel != 0:
    #     for_back_vel_l = 0
    #     for_back_vel_r = 0

if __name__ == '__main__':
    try:
        rospy.init_node('joy_control', anonymous=True)

        fl_joy_pub = rospy.Publisher("/SP_FL", Float64, queue_size=10)
        fr_joy_pub = rospy.Publisher("/SP_FR", Float64, queue_size=10)
        s2s_joy_pub = rospy.Publisher("/SP_S2S", Float64, queue_size=10)
        vl_joy_pub = rospy.Publisher("/SP_VL", Float64, queue_size=10)
        vr_joy_pub = rospy.Publisher("/SP_VR", Float64, queue_size=10)
        vc_joy_pub = rospy.Publisher("/SP_VC", Float64, queue_size=10)

        depth_setpoint_pub = rospy.Publisher("/depth_setpoint", Float64, queue_size=10)
        depth_lock_pub = rospy.Publisher("/DL_bool", Bool, queue_size=10)

        state_subscriber = rospy.Subscriber("/sensor", Sensor, depth_state_cb)
        joy_sub = rospy.Subscriber("/joy", Joy, joystick_callback)
        
        while not rospy.is_shutdown():

            fl_joy_pub.publish(for_back_vel_l)
            fr_joy_pub.publish(for_back_vel_r)
            s2s_joy_pub.publish(side_vel)
            vl_joy_pub.publish(vertical_vel)
            vr_joy_pub.publish(vertical_vel)
            vc_joy_pub.publish(pitch_vel)
            depth_lock_pub.publish(depth_lock)
            
            loop_rate = rospy.Rate(10)
            loop_rate.sleep()

            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
