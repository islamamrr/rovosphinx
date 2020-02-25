#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ros_essentials_cpp.msg import Thrust_vel
from std_msgs.msg import Float64
import funcs

fr_vel = fl_vel = s2s_vel = vr_vel = vl_vel = vc_vel = 0

def fr_joy_callback(input):
    global fr_vel
    
    fr_vel = round(input.data, 2)
    
def fl_joy_callback(input):
    global fl_vel

    fl_vel = round(input.data, 2)

def s2s_joy_callback(input):
    global s2s_vel

    s2s_vel = round(input.data, 2)

def vr_joy_callback(input):
    global vr_vel

    vr_vel = round(input.data, 2)

def vl_joy_callback(input):
    global vl_vel

    vl_vel = round(input.data, 2)

def vc_joy_callback(input):
    global vc_vel
    
    vc_vel = round(input.data, 2)

def move_thrusters(fr_vel, fl_vel, s2s_vel, vr_vel, vc_vel, vl_vel):

    all_vels = Thrust_vel()
    #map the values from the joystick to values that can be handled by the thrusters
    
    #forward thrusters
    if fr_vel > 0:
        all_vels.fr = funcs.map_val(fr_vel, 0, 1, 0, 400)
        if all_vels.fr > 400:
            all_vels.fr = 400        
    elif fr_vel < 0:
        all_vels.fr = funcs.map_val(fr_vel, 0, -1, 0, 400) * -1
        if all_vels.fr < -400:
            all_vels.fr = -400        
    else:
        all_vels.fr = 0

    if fl_vel > 0:    
        all_vels.fl = funcs.map_val(fl_vel, 0, 1, 0, 400)
        if all_vels.fl > 400:
            all_vels.fl = 400
    elif fl_vel < 0:    
        all_vels.fl = funcs.map_val(fl_vel, 0, -1, 0, 400) * -1
        if all_vels.fl < -400:
            all_vels.fl = -400
    else:
        all_vels.fl = 0

    #side thruster
    if s2s_vel > 0:
        all_vels.s2s = funcs.map_val(s2s_vel, 0, 1, 0, 400)
        if all_vels.s2s > 400:
            all_vels.s2s = 400        
    elif s2s_vel < 0:
        all_vels.s2s = funcs.map_val(s2s_vel, 0, -1, 0, 400) * -1
        if all_vels.s2s < -400:
            all_vels.s2s = -400        
    else:
        all_vels.s2s = 0

    #vertical thrusters
    if vr_vel > 0:
        all_vels.vr = funcs.map_val(vr_vel, 0, 1, 0, 400)
        if all_vels.vr > 400:
            all_vels.vr = 400        
    elif vr_vel < 0:
        all_vels.vr = funcs.map_val(vr_vel, 0, -1, 0, 400) * -1
        if all_vels.vr < -400:
            all_vels.vr = -400        
    else:
        all_vels.vr = 0

    if vl_vel > 0:    
        all_vels.vl = funcs.map_val(vl_vel, 0, 1, 0, 400)
        if all_vels.vl > 400:
            all_vels.vl = 400        
    elif vl_vel < 0:    
        all_vels.vl = funcs.map_val(vl_vel, 0, -1, 0, 400) * -1
        if all_vels.vl < -400:
            all_vels.vl = -400        
    else:
        all_vels.vl = 0

    if vc_vel > 0:    
        all_vels.vc = funcs.map_val(vc_vel, 0, 1, 0, 400)
        if all_vels.vc > 400:
            all_vels.vc = 400        
    elif vc_vel < 0:    
        all_vels.vc = funcs.map_val(vc_vel, 0, -1, 0, 400) * -1
        if all_vels.vc < -400:
            all_vels.vc = -400
    else:
        all_vels.vc = 0        

    loop_rate = rospy.Rate(10)
    velocity_publisher.publish(all_vels)

    loop_rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('thrust_ctrl', anonymous=True)

        velocity_publisher = rospy.Publisher("/thrusters_vels", Thrust_vel, queue_size=10)

        fr_joy_sub = rospy.Subscriber("/MotorHR/state", Float64, fr_joy_callback)
        fl_joy_sub = rospy.Subscriber("/MotorHL/state", Float64, fl_joy_callback)
        s2s_joy_sub = rospy.Subscriber("/MotorS2S/state", Float64, s2s_joy_callback)
        vc_joy_sub = rospy.Subscriber("/MotorVC/state", Float64, vc_joy_callback)
        vl_joy_sub = rospy.Subscriber("/MotorVL/state", Float64, vl_joy_callback)
        vr_joy_sub = rospy.Subscriber("/MotorVR/state", Float64, vr_joy_callback)
                
        while not rospy.is_shutdown():

            move_thrusters(fr_vel, fl_vel, s2s_vel, vr_vel, vc_vel, vl_vel)

            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")