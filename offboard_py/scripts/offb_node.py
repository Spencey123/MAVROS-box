#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")


    pose = PoseStamped()
    nowPose = PoseStamped()

    def local_position_callback(data):
        global nowPose
        global pose
        global current_state
        global offb_set_mode
        global arm_cmd
        nowPose = data
        if(current_state.mode != "OFFBOARD"):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            
        else:
            if(not current_state.armed):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        ##new addition for land when pose close to right pose check how to see if position of drone is published.
            else:
                if  (pose != nowPose):
                    local_pos_pub.publish(pose)
                    print(nowPose.pose.position.x, nowPose.pose.position.y, nowPose.pose.position.z)
                else :
                    offb_set_mode.custom_mode = 'AUTO.LAND'

    state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("/mavros/local_position/local", PoseStamped, local_position_callback, queue_size=10)
    
    
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
    pose.pose.position.x = 2
    pose.pose.position.y = -5
    pose.pose.position.z = 3

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
    rospy.spin()
    '''
    while(not rospy.is_shutdown()):
        


        rate.sleep()

    '''