#! /usr/bin/env python

from math import sin, cos, pi
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
desired_height = 1
boxsize = 10
current_coord = (0,0) #initialize 1st position
radius = 8
num_of_points = 20


def get_coordinates_square(boxsize):
    center_coordinate = (0,0)
    BL= (-boxsize,-boxsize)
    BR= (boxsize,-boxsize)
    TR= (boxsize, boxsize)
    TL= (-boxsize,boxsize)
    return (BL,BR,TR,TL)

def get_coordinates_circle(radius, num_of_points):
    #eqn for point on circle = radius*sin(360/num point),radius*cos(360/num of points)
    points = []
    for i in range(num_of_points):
        points.append(((radius*sin(2*i*pi/num_of_points)),(radius*cos(2*i*pi/num_of_points))))
    return tuple(points)


def get_new_coord(current_coord):
    if current_coord == (0,0):
        return get_coordinates_circle(radius,num_of_points)[0] #go to BL
        
    else :
        for i in range(len(get_coordinates_circle(radius,num_of_points))):
            if current_coord == get_coordinates_circle(radius,num_of_points)[num_of_points-1]:
                return get_coordinates_circle(radius,num_of_points)[0]
            elif current_coord == get_coordinates_circle(radius,num_of_points)[i]:
                return (get_coordinates_circle(radius,num_of_points)[i+1])

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
    pose.pose.position.x = current_coord[0]
    pose.pose.position.y = current_coord[1]
    pose.pose.position.z = desired_height

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

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

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
            current_coord = get_new_coord(current_coord)
            pose.pose.position.x = current_coord[0]
            pose.pose.position.y = current_coord[1]
            pose.pose.position.z = desired_height
            print("reached")
            print (current_coord)


        else :
            pose.pose.position.x = current_coord[0]
            pose.pose.position.y = current_coord[1]
            pose.pose.position.z = desired_height
            local_pos_pub.publish(pose)
            
            
            
        rate.sleep()
    rospy.spin()
