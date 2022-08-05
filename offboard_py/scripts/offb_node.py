#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def local_position_callback(data):
    global nowPose
    nowPose = data


def isclose(a,b,abs_tol):
    return(abs(a-b) <= abs_tol)

    
if __name__ == "__main__":
    rospy.init_node("offb_node_py")


    pose = PoseStamped()
    nowPose = Odometry()

    state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, callback = local_position_callback, queue_size=10)
    
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()


    #set goal for movement from init
    pose.pose.position.x = 3
    pose.pose.position.y = -5
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")
            
            last_req = rospy.Time.now()

        ##new addition for land when pose close to right pose check how to see if position of drone is published.
        
        elif  isclose(pose.pose.position.x,nowPose.pose.pose.position.x,0.1)&\
                isclose(pose.pose.position.y,nowPose.pose.pose.position.y,0.1)&\
                isclose(pose.pose.position.z,nowPose.pose.pose.position.z,0.1):
            offb_set_mode.custom_mode = 'AUTO.LAND'
        else :
            local_pos_pub.publish(pose)
            print((nowPose.pose.pose.position),(pose.pose.position))
        
        
        
        rate.sleep()
    rospy.spin()
