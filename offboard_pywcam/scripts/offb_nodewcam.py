#! /usr/bin/env python
from cmath import sin
from math import asin, atan2, degrees, radians, sinh
from os import close
from pickle import FALSE
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


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
        self.coll = []
        self.landpos = [(0.5,1.5)]
        self.fly()


    def state_cb(self,msg):
        self.current_state = msg


    def local_position_callback(self,data):
        self.nowPose = data
    

    def path(self):
        return [(0,0),(7,0),(7,1.5),(5,2),(5,3),(5,1)]


    def get_latest_path(self):
        global num_of_points_in_path 
        num_of_points_in_path = len(self.path())
        return self.path()
    

    def isclose(self,a,b,abs_tol):
        return(abs(a-b) <= abs_tol)


    def get_new_goal_coord(self):
        if self.coordinate_count == 0:
            self.coordinate_count += 1
            return self.get_latest_path()[0] #go to BL
            
        elif self.coordinate_count == num_of_points_in_path:
            self.landing()
            return self.get_latest_path()[self.coordinate_count-1]

        else:    
            self.coordinate_count += 1
            print ("incrementing")
            return self.get_latest_path()[self.coordinate_count-1]


    def scan_obstacle(self,msg):
    #print(len(msg.ranges))          #len is 360
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
        for i in range(len(self.scann.ranges)):
            if 0.6 <= (self.scann.ranges[i]) <= 8:      #filter bad data from min to max range
                self.coll.append((round(self.scann.ranges[i],3),i))
        self.pub.publish(self.scann)


    def landing(self):
        while self.goalpose.pose.position.z != 0 :
            self.offb_set_mode = SetModeRequest()
            self.offb_set_mode.custom_mode = 'AUTO.LAND'
            print("Landing")
            self.goalpose.pose.position.z = 0    
        close()

    def bang(self):
       #collision detection at 0.7m from sensor. reason why is corner of 0.5,0.5.
        if(len(self.coll) != 0):
            for i in self.coll :
                if i[0]<=0.65:
                    print("bang")
                    self.landing()
        
    
    def avoid(self):
        return
        

    #main avoidance
    #if no collison, post waypoint within 5m from drone in line to goalpoint
    def check_coll(self):
        if self.get_point() != None :
            (a,b,mag,traj_angle)=self.get_point()
            if self.coll != []:
                for i in self.coll:
                    angle_of_care = abs(traj_angle-i[1])
                    if angle_of_care > 180:
                        angle_of_care = 360 - angle_of_care
                    #print (traj_angle,i[1],angle_of_care)
                    if angle_of_care<90:
                        perpendicular_distance_to_path = sin(radians(angle_of_care)/radians(i[0]))
                        #print(perpendicular_distance_to_path)
                    print(angle_of_care)
            return(a,b)
            '''
            for i in self.coll:
                if i[0] <= mag:
                    #print(degrees(sinh(0.75/i[0])))
                    #print (abs(i[1] - angle) - int(degrees(asin(0.71/i[0])))/5,i[0],mag)
                    if abs(i[1] - angle) < int(degrees(asin(0.71/i[0])))/5:
                        print("collision path detected")
            return (a,b)
            '''
        else:
            return (0,0)
    

    #get_point is to check if way forward is clear. If clear for the next 5m mark clear and check every cycle. if goal is within 5m then publish goal. if further than 5m publish 5m forward. 
    def get_point(self):
        if (((self.goalpose.pose.position.x - self.nowPose.pose.pose.position.x) != 0.0) & ((self.goalpose.pose.position.y - self.nowPose.pose.pose.position.y) != 0.0)):
            # find the vector a-b
            abx, aby = (self.goalpose.pose.position.x - self.nowPose.pose.pose.position.x, self.goalpose.pose.position.y - self.nowPose.pose.pose.position.y)
            # find the length of this vector
            len_ab = (abx * abx + aby * aby) ** 0.5
            # find the angle of this vector in the space I made
            angle_of_vector = degrees(atan2(abx,aby))
            #convert angle to 0-360 0W rplidar scan
            angle_of_vector1 = 270 - angle_of_vector
            if angle_of_vector1 > 360:
                angle_of_vector1 = abs(angle_of_vector1-360)
            #angle_of_vector_in_revised_scan = int(angle_of_vector+90)/5
            if (len_ab <= 5.8):
                return (self.goalpose.pose.position.x,self.goalpose.pose.position.y,len_ab,angle_of_vector1)
            # scale to length 5
            else:
                ab5x, ab5y = abx *5.8 / len_ab, aby *5.8 / len_ab
            # add to a (== p0)
                finx, finy = self.nowPose.pose.pose.position.x + ab5x, self.nowPose.pose.pose.position.y + ab5y
                return (finx,finy,len_ab,angle_of_vector1)        
        
      
    def fly(self): 

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode) 
        self.set_mode_client.call()

        last_req = rospy.Time.now()

        #set goal for movement from init
        self.goalpose.pose.position.x = self.current_coord[0]
        self.goalpose.pose.position.y = self.current_coord[1]
        self.goalpose.pose.position.z = self.desired_height

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        self.offb_set_mode = SetModeRequest()

        if self.offb_set_mode.custom_mode == '':
            self.offb_set_mode.custom_mode = 'OFFBOARD'

        rate = rospy.Rate(20)

        while(not rospy.is_shutdown() and self.offb_set_mode.custom_mode != "AUTO.LAND"):
            self.avoid()
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
                self.goalpose.pose.position.x = x
                self.goalpose.pose.position.y = y
                self.local_pos_pub.publish(self.goalpose)

        rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node("offb_node_pywcam")
    drone = Drone(1)
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
