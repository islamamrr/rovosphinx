#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ros_essentials_cpp.msg import Sensor
from ros_essentials_cpp.msg import Thrust_vel
from std_msgs.msg import Float64
from std_msgs.msg import Bool
# from ros_essentials_cpp.msg import TwoCentres
import funcs

curr_joy = Joy()
curr_joy.axes = [0,0,0,0,0,0,0,0]
curr_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
L3_X, L3_Y, L2, R3_X, R3_Y, R2 = 0, 1, 2, 3, 4, 5
L1, R1 = 4, 5
depth = 0.0
control = 0.0
DL_bool = 0
fr_vel = fl_vel = s2s_vel = vr_vel = vl_vel = vc_vel = 0

def fr_joy_callback(input):
    global fr_vel
    
    fr_vel = round(input.data, 2)
    
def fl_joy_callback(input):
    global fl_vel

    fl_vel = round(input.data, 2)

def s2s_joy_callback(input):
    global s2s_vel

    #opposite sign to compensate for error
    s2s_vel = -round(input.data, 2)

def vr_joy_callback(input):
    global vr_vel

    #opposite sign to compensate for error
    vr_vel = -round(input.data, 2)

def vl_joy_callback(input):
    global vl_vel

    vl_vel = round(input.data, 2)

def vc_joy_callback(input):
    global vc_vel
    
    vc_vel = round(input.data, 2)

def depth_state_cb(sensor_message):
    global depth

    depth = sensor_message.Depth

def DL_callback(input):
    global DL_bool
    
    DL_bool = input.data

def pid_depth_cb(input):
    global control

    control = input.data

def joystick_callback(joy):
    global curr_joy

    curr_joy = joy

def move_thrusters(fr_vel, fl_vel, s2s_vel, vr_vel, vc_vel, vl_vel):

    all_vels = Thrust_vel()
    #map the values from the joystick to values that can be handled by the thrusters
    
    #forward thrusters
    if fr_vel > 0:
        all_vels.fr = funcs.map_val(fr_vel, 0, 1, 0, 200)
        if all_vels.fr > 200:
            all_vels.fr = 200        
    elif fr_vel < 0:
        all_vels.fr = funcs.map_val(fr_vel, 0, -1, 0, 200) * -1
        if all_vels.fr < -200:
            all_vels.fr = -200        
    else:
        all_vels.fr = 0

    if fl_vel > 0:    
        all_vels.fl = funcs.map_val(fl_vel, 0, 1, 0, 200)
        if all_vels.fl > 200:
            all_vels.fl = 200
    elif fl_vel < 0:    
        all_vels.fl = funcs.map_val(fl_vel, 0, -1, 0, 200) * -1
        if all_vels.fl < -200:
            all_vels.fl = -200
    else:
        all_vels.fl = 0

    #to avoid steady-state error at zero caused by PID 
    if curr_joy.axes[L3_Y] == 0 and curr_joy.buttons[R1] == 0 and curr_joy.buttons[L1] == 0:
        all_vels.fr = all_vels.fl = 0


    #side thruster
    if s2s_vel > 0:
        all_vels.s2s = funcs.map_val(s2s_vel, 0, 1, 0, 200)
        if all_vels.s2s > 200:
            all_vels.s2s = 200        
    elif s2s_vel < 0:
        all_vels.s2s = funcs.map_val(s2s_vel, 0, -1, 0, 200) * -1
        if all_vels.s2s < -200:
            all_vels.s2s = -200        
    else:
        all_vels.s2s = 0

    #to avoid steady-state error at zero caused by PID 
    if curr_joy.axes[L3_X] == 0:
        all_vels.s2s = 0


    #vertical thrusters
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

    #if DL is activated the 2 vertical thrusters vr & vl will be controlled by the pid
    if DL_bool == 1:
        all_vels.vr = all_vels.vl = control
    else:
        disable_publisher.publish(0)

        if vr_vel > 0:
            all_vels.vr = funcs.map_val(vr_vel, 0, 1, 0, 400)
            if all_vels.vr > 300:
                all_vels.vr = 300        
        elif vr_vel < 0:
            all_vels.vr = funcs.map_val(vr_vel, 0, -1, 0, 400) * -1
            if all_vels.vr < -300:
                all_vels.vr = -300        
        else:
            all_vels.vr = 0

        if vl_vel > 0:    
            all_vels.vl = funcs.map_val(vl_vel, 0, 1, 0, 400)
            if all_vels.vl > 300:
                all_vels.vl = 300        
        elif vl_vel < 0:    
            all_vels.vl = funcs.map_val(vl_vel, 0, -1, 0, 400) * -1
            if all_vels.vl < -300:
                all_vels.vl = -300        
        else:
            all_vels.vl = 0   

        #to avoid steady-state error at zero caused by PID
        if curr_joy.axes[R2] == 1 and curr_joy.axes[L2] == 1:
            all_vels.vr = all_vels.vl = 0
            if curr_joy.axes[R3_Y] == 0:
                all_vels.vc = 0

            

    loop_rate = rospy.Rate(10)
    velocity_publisher.publish(all_vels)
    state_publisher.publish(depth)
    loop_rate.sleep()

    # print curr_joy.buttons[3]

if __name__ == '__main__':
    try:
        rospy.init_node('thrust_ctrl', anonymous=True)

        velocity_publisher = rospy.Publisher("/thrusters_vels", Thrust_vel, queue_size=10)
        state_publisher = rospy.Publisher("/state", Float64, queue_size=10)
        disable_publisher = rospy.Publisher("/Depth_control/control_effort", Float64, queue_size=10)
        #setpoint_publisher = rospy.Publisher("/setpoint", Float64, queue_size=10)


        fr_joy_sub = rospy.Subscriber("/MotorFR/state", Float64, fr_joy_callback)
        fl_joy_sub = rospy.Subscriber("/MotorFL/state", Float64, fl_joy_callback)
        s2s_joy_sub = rospy.Subscriber("/MotorS2S/state", Float64, s2s_joy_callback)
        vc_joy_sub = rospy.Subscriber("/MotorVC/state", Float64, vc_joy_callback)
        vl_joy_sub = rospy.Subscriber("/MotorVL/state", Float64, vl_joy_callback)
        vr_joy_sub = rospy.Subscriber("/MotorVR/state", Float64, vr_joy_callback)

        joy_sub = rospy.Subscriber("/joy", Joy, joystick_callback)
        state_subscriber = rospy.Subscriber("/sensor", Sensor, depth_state_cb)
        control_effort_subscriber = rospy.Subscriber("/Depth_control/state", Float64, pid_depth_cb) 

        depth_lock_sub = rospy.Subscriber("/DL_bool", Bool, DL_callback)

                
        while not rospy.is_shutdown():

            move_thrusters(fr_vel, fl_vel, s2s_vel, vr_vel, vc_vel, vl_vel)

            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")