#! /usr/bin/env python
from cmath import cos, sin
from math import asin, atan2, degrees, radians, sinh
from numbers import Real
from os import close
from pickle import FALSE, TRUE
from turtle import clear
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


'''
Credits: Arnab cuz he smart, Aaron for helping find global pose, ZC for allowing me in, Hansel for doing this wayyy before anyone else.
Tangent bug algorithm used as described in paper below
https://www.ijert.org/research/comparison-of-various-obstacle-avoidance-algorithms-IJERTV4IS120636.pdf

Idea is for a drone to follow waypoints posted in a list of tuples [(x1,y1),(x2,y2),(x3,y3)] upon moving to an obstacle, navigate around the obstacle and continue along. When at final point, land.

Caveats, for now, waypoints must be outside of the obstacle. There is no knowledge of whether the drone has moved to the right waypoint if the waypoint is within the obstacle.

Obstacle file is loaded on new_world.sdf in local .ros /home/spencer/PX4-Autopilot/Tools/sitl_gazebo/worlds
'''

class Drone:
    def __init__(self,desired_height):      #iris drone size = 0.9,0.9,0.15m l,w,h bang at 0.7, clearance at 0.8
        self.current_state = State()
        self.desired_height = desired_height
        self.current_coord = (0,0) #initialize 1st position
        self.coordinate_count = 0
        self.goalpose = PoseStamped()
        self.nowPose = Odometry()
        self.nextpose = PoseStamped()
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, callback = self.local_position_callback, queue_size=10)
        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.scan_obstacle)
        self.pub = rospy.Publisher('/laser/revised_scan', LaserScan, queue_size = 10)
        self.scann = LaserScan()
        self.traj_angle = 0
        self.coll = []
        self.avoid_scale = 2
        self.landpos = [(0.5,1.5)]
        self.fly()


##path is given here
    def path(self):
        return [(6,3),(0,3),(3,3)]


    def state_cb(self,msg):
        self.current_state = msg


    def local_position_callback(self,data):
        self.nowPose = data
    

    def get_latest_path(self):
        global num_of_points_in_path 
        num_of_points_in_path = len(self.path())
        return self.path()
    

    def isclose(self,a,b,abs_tol):
        return(abs(a-b) <= abs_tol)


    def get_new_goal_coord(self):            
        if self.coordinate_count >= num_of_points_in_path -1 :
            self.landing()
            return self.get_latest_path()[-1]

        else:    
            self.coordinate_count += 1
            print ("incrementing")
            print (self.coordinate_count,num_of_points_in_path)
            return self.get_latest_path()[self.coordinate_count]


    def scan_obstacle(self,msg):
        current_time = rospy.Time.now()
        self.scann.header.stamp = current_time
        self.scann.header.frame_id = 'laser'
        self.scann.angle_min = -3.1415
        self.scann.angle_max = 3.1415
        self.scann.angle_increment = 0.00311202858575
        self.scann.time_increment = 4.99999987369e-05
        self.scann.range_min = 0.50999999977648
        self.scann.range_max = 32.0
        self.scann.ranges = msg.ranges
        self.coll = []
        self.coll_angles = []
        for i in range(len(self.scann.ranges)):
            if 0.6 <= (self.scann.ranges[i]) <= 8:      #filter bad data from min to max range
                self.coll.append((round(self.scann.ranges[i],3),i))
                self.coll_angles.append(i)
        self.pub.publish(self.scann)


    def landing(self):
        if self.nowPose.pose.pose.position.z >= .1 :
            self.offb_set_mode = SetModeRequest()
            self.offb_set_mode.custom_mode = 'AUTO.LAND'
            print("Landing")
            self.goalpose.pose.position.z = 0    
        else:
            close() 

    def bang(self):
       #collision detection at 0.7m from sensor. reason why is corner of 0.5,0.5.
        if(len(self.coll) != 0):
            for i in self.coll :
                if i[0]<=0.65:
                    print("bang")
                    self.landing()
        

    #get_point is to check if way forward is clear. If clear for the next 5.8m mark clear and check every cycle. if goal is within 5.8m then publish goal. if further than 5.8m publish 5.8m forward. 
    def get_point(self):
        if (((self.goalpose.pose.position.x - self.nowPose.pose.pose.position.x) != 0.0) & ((self.goalpose.pose.position.y - self.nowPose.pose.pose.position.y) != 0.0)):
            # find the vector a-b
            abx, aby = (self.goalpose.pose.position.x - self.nowPose.pose.pose.position.x, self.goalpose.pose.position.y - self.nowPose.pose.pose.position.y)
            # find the length of this vector
            len_ab = (abx * abx + aby * aby) ** 0.5
            # find the angle of this vector in the space I made
            angle_of_vector = int(degrees(atan2(abx,aby)))
            #convert angle to 0-360 0W rplidar scan
            self.traj_angle = 270 - angle_of_vector
            if self.traj_angle > 360:
                self.traj_angle = abs(self.traj_angle-360)
            #angle_of_vector_in_revised_scan = int(angle_of_vector+90)/5
            if (len_ab <= 5.8):
                return (self.goalpose.pose.position.x,self.goalpose.pose.position.y,len_ab,self.traj_angle)
            # scale to length 5
            else:
                ab5x, ab5y = abx *5.8 / len_ab, aby *5.8 / len_ab
            # add to a (== p0)
                finx, finy = self.nowPose.pose.pose.position.x + ab5x, self.nowPose.pose.pose.position.y + ab5y
                return (finx,finy,len_ab,self.traj_angle)        
        

    #main avoidance
    #if no collison, post waypoint within 5m from drone in line to goalpoint
    def check_coll(self):
        if self.get_point() != None :
            (a,b,self.mag,self.traj_angle)=self.get_point()
            if self.coll != []:
                for i in self.coll:
                    angle_of_care = abs(self.traj_angle-i[1])
                    if angle_of_care > 180:
                        angle_of_care = 360 - angle_of_care
                    #print (traj_angle,i[1],angle_of_care)
                    if angle_of_care < 90 :
                        perpendicular_distance_to_path = sin(radians(angle_of_care))*i[0]          
                        if perpendicular_distance_to_path.real < 0.8 and i[0]< self.mag:
                            a,b = self.avoid2(self.traj_angle,i[1],i[0]) #inputs for avoid: traj, obstacle, obstacle distance
                            return (a,b)
                            
                    else:
                        return self.get_latest_path()[self.coordinate_count]
            return self.get_latest_path()[self.coordinate_count]
        else:
            return self.get_latest_path()[self.coordinate_count]


    
    #if collide, always go left around obstacle
    def get_clear_angle(self,int):
        if int not in self.coll_angles:
            if int == 0 :
                return (self.scann.ranges[360],int)
            else :
                return (self.scann.ranges[int-1],int)
        else:
            if int == 360:
                return self.get_clear_angle (0)
            else:
                return (self.get_clear_angle(int + 1))
    

#sort list in ascending order with min and max as tuples
    def sort_sensor(self,int,obstacle_angle):
        if int in self.coll_angles:
            b = []
            subList = []
            prev_n = -1
            for n in self.coll_angles:
                if prev_n+1 != n:            # end of previous subList and beginning of next
                    if subList:              # if subList already has elements
                        b.append(subList)
                        subList = []
                subList.append(n)
                prev_n = n
            if subList:
                b.append(subList)           #b is sorted list i.e [[1,2],[4,5],[6,7]]
            print (b[0][0],b[-1][-1],"c")
            if b[0][0] == 0 and b[-1][-1]==359:
                c = b[-1]
                d = b[0]
                e = []
                for i in b[0]:
                    c.append(i+360)
                for i in b[-1]:
                    e.append(i-360)
                b[0]=e
                b[-1]=c
            print (b,2)
            for i in b:
                if (int in i):
                    #print(int - i[1] , i[-1] - int)
                    if int - i[1] >= i[-1] - int:
                        print(i[-1],True,1)
                        return (i[-1],True)
                    else:
                        print(i[1],False,2)
                        return (i[1],False)
        else:
            if int - obstacle_angle >180: #0-360 scenario
                return (360 - int - obstacle_angle,False)
            if int - obstacle_angle < - 180:
                return (360 + int - obstacle_angle, True)
                #write code catching 0-360 transition. 
            if int > obstacle_angle:
                print (int,obstacle_angle,1)
                return(int,True)
            else:
                print(int,obstacle_angle,0)
                return(int,False)

        
    
    def avoid(self,traj,obstacle_angle,ob_dist): #get nearest clear point closest to the goal. 1st step get first point off angle
        clear_angle = self.get_clear_angle(self.traj_angle)
        avoid_angle = clear_angle[1] + 90
        #print (clear_angle,traj,obstacle_angle,ob_dist, avoid_angle,3)
        x_ob,y_ob,x_avoid,y_avoid =  -ob_dist*cos(radians(clear_angle[1])),-ob_dist*sin(radians(clear_angle[1])), -self.avoid_scale*cos(radians(avoid_angle)),-self.avoid_scale*sin(radians(avoid_angle))
        a,b = (self.nowPose.pose.pose.position.x + x_ob + x_avoid, self.nowPose.pose.pose.position.y + y_ob + y_avoid)
        #print (a,b,self.nowPose.pose.pose.position.x, self.nowPose.pose.pose.position.y, x_ob.real,y_ob.real,x_avoid.real,y_avoid.real,"d")
        return(a.real,b.real)
    

    def avoid2(self,traj,obstacle_angle,ob_dist): #get nearest clear point closest to the goal. 1st step get first point off angle
        best_angle,moving_L = self.sort_sensor(self.traj_angle,obstacle_angle)
        print (best_angle, moving_L)
        if moving_L == True:
            avoid_angle = best_angle + 90
        else:
            avoid_angle = best_angle - 90
        #print (clear_angle,traj,obstacle_angle,ob_dist, avoid_angle,3)
        x_ob,y_ob,x_avoid,y_avoid =  -ob_dist*cos(radians(best_angle)),-ob_dist*sin(radians(best_angle)), -self.avoid_scale*cos(radians(avoid_angle)),-self.avoid_scale*sin(radians(avoid_angle))
        a,b = (self.nowPose.pose.pose.position.x + x_ob + x_avoid, self.nowPose.pose.pose.position.y + y_ob + y_avoid)
        #print (a,b,self.nowPose.pose.pose.position.x, self.nowPose.pose.pose.position.y, x_ob.real,y_ob.real,x_avoid.real,y_avoid.real,"d")
        return(a.real,b.real)


    def fly(self): 

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode) 
        self.set_mode_client.call()

        last_req = rospy.Time.now()

        #set goal for movement from init
        self.goalpose.pose.position.x = self.get_latest_path()[self.coordinate_count][0]
        self.goalpose.pose.position.y = self.get_latest_path()[self.coordinate_count][1]
        self.goalpose.pose.position.z = self.desired_height

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        self.offb_set_mode = SetModeRequest()

        if self.offb_set_mode.custom_mode == '':
            self.offb_set_mode.custom_mode = 'OFFBOARD'

        rate = rospy.Rate(20)

        while(not rospy.is_shutdown() and self.offb_set_mode.custom_mode != "AUTO.LAND"):
            self.bang()

            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0) and self.current_state.mode != "AUTO.LAND"):  #check offboard mode on
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    print("OFFBOARD enabled")
                last_req = rospy.Time.now()

            elif(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0) and self.current_state.mode != "AUTO.LAND"):         #check if vehicle armed
                if(arming_client.call(arm_cmd).success == True):
                    print("Vehicle armed")
                last_req = rospy.Time.now()

            elif (self.isclose(self.goalpose.pose.position.x,self.nowPose.pose.pose.position.x,0.1)&\
                self.isclose(self.goalpose.pose.position.y,self.nowPose.pose.pose.position.y,0.1)&\
                self.isclose(self.goalpose.pose.position.z,self.nowPose.pose.pose.position.z,0.1)): #check if position reached goal position for new goal, to change : insert data for next body before moving on
                self.current_coord = self.get_new_goal_coord()
                self.goalpose.pose.position.x = self.current_coord[0]
                self.goalpose.pose.position.y = self.current_coord[1]
                self.goalpose.pose.position.z = self.desired_height
                print("reached")
                print (self.current_coord)
                print (self.coordinate_count)

            else :                                                                                              #keeps publishing goal position for FC to follow
                (x,y)=self.check_coll()
                self.nextpose.pose.position.x = x
                self.nextpose.pose.position.y = y
                self.nextpose.pose.position.z = self.desired_height
                self.local_pos_pub.publish(self.nextpose)
                #print(x,y,self.coordinate_count,"g")
                

        rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node("offb_node_pywcam")
    drone = Drone(3)
    rospy.spin()

'''
EXAMPLE CODE
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
    def callback_number(self, msg):
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)
    def callback_reset_counter(self, req):
        if req.data:
            self.counter = 0
            return True, "Counter has been successfully reset"
        return False, "Counter has not been reset"
if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()

'''
