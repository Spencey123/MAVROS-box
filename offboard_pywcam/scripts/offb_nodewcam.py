#! /usr/bin/env python

from math import sin, cos, pi
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class Drone:
    def __init__(self,desired_height, radius, num_of_points,length,girth): 
        self.current_state = State()
        self.desired_height = desired_height
        self.boxsize = 10
        self.current_coord = (0,0) #initialize 1st position
        self.radius = radius
        self.num_of_points = num_of_points
        self.offset_for_newcircle = 0.1 #starting and end loop for circles are same so it iterates back to for the starting circling of infinity.
        self.length = length
        self.girth = girth
        self.coordinate_count = 0
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, callback = self.local_position_callback, queue_size=10)
        self.landpos = [(0.5,1.5)]
        self.fly()

    def get_coordinates_square(self,boxsize):
        BL= (-boxsize,-boxsize)
        BR= (boxsize,-boxsize)
        TR= (boxsize, boxsize)
        TL= (-boxsize,boxsize)
        return (BL,BR,TR,TL)

    def get_coordinates_circle(self,radius, num_of_points,offsetx,offsety):
        #eqn for point on circle = radius*sin(360/num point),radius*cos(360/num of points)
        points = []
        for i in range(num_of_points):
            points.append(((offsetx + radius*sin(2*i*pi/num_of_points)),(offsety + radius*cos(2*i*pi/num_of_points))))
        return points

    def get_coordinates_infinity(self,radius, num_of_points):
        #eqn for circle gets list of points to 1 circle 
        #get equation for 2nd circle above first, add all points on 2nd
        #bisect lists in 2 and join as 1stlist, mid2nd-top point and back. 
        btm_circle = self.get_coordinates_circle(radius,num_of_points,0,-radius)
        top_circle = self.get_coordinates_circle(radius,num_of_points,0,+radius) #shift circle 2r+small amount up
        right_tune_circle = []
        left_tune_circle = []
        coord_list_infinity =[]

        for i in range(num_of_points):
            if (i > 0.5* num_of_points):
                left_tune_circle.insert(int(num_of_points/2)-i,top_circle[i])
            else :
                right_tune_circle.insert(i,top_circle[i])
        right_tune_circle.reverse()             #flip right side to start circle from bottom to top
        coord_list_infinity = btm_circle + right_tune_circle + left_tune_circle #combine circles together
        return coord_list_infinity

    def get_coordinates_dong_curve(self, length, girth, num_of_points):
        points = []
        for i in range(num_of_points):
            if (num_of_points % 2 == 0):
                if (i<0.5*num_of_points+1):
                    points.append(((length+ girth *sin(2*i*pi/(num_of_points))),(girth*cos(2*i*pi/(num_of_points)))))
            else:
                if (i<0.5*num_of_points):
                    points.append(((length+ girth *sin(2*i*pi/(num_of_points))),(girth*cos(2*i*pi/(num_of_points)))))
        return points

    def dong_draw (self, radius,num_of_points,length, girth):
        balls = self.get_coordinates_infinity(radius,num_of_points)
        full_dong_curve = self.get_coordinates_dong_curve(length, girth, num_of_points)
        fix_ball_curve = [(0.0,0.0)]
        for i in range(len(balls)):
            if (0.5*len(balls)<i<0.625*len(balls)):
                fix_ball_curve.append(balls[i])
        first_point = [(radius,radius)]
        last_point = [(radius,-radius)]
        final_draw = balls + fix_ball_curve + first_point + full_dong_curve + last_point + self.landpos
        return (final_draw)


    def get_latest_path(self, radius, num_of_points,length,girth):
        global num_of_points_in_path 
        num_of_points_in_path = len(self.dong_draw(radius,num_of_points,length, girth))
        return self.dong_draw(radius,num_of_points,length, girth)
    
    def isclose(self,a,b,abs_tol):
        return(abs(a-b) <= abs_tol)

    def landing(self):
        self.offb_set_mode.custom_mode = 'AUTO.LAND'
        self.goalpose.pose.position.z = 0

    def get_new_coord(self,current_coord):
        if self.coordinate_count == 0:
            self.coordinate_count += 1
            return self.get_latest_path(self.radius,self.num_of_points,self.length,self.girth)[0] #go to BL
            
        elif self.coordinate_count == num_of_points_in_path:
            self.landing()
            return self.get_latest_path(self.radius,self.num_of_points,self.length,self.girth)[self.coordinate_count-1]

        else:    
            self.coordinate_count += 1
            print ("incrementing")
            return self.get_latest_path(self.radius,self.num_of_points,self.length,self.girth)[self.coordinate_count-1]


    def state_cb(self,msg):
        self.current_state = msg

    def local_position_callback(self,data):
        self.nowPose = data
       
    def fly(self): 

        self.goalpose = PoseStamped()
        self.nowPose = Odometry()

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
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        rate = rospy.Rate(20)
        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):  #check offboard mode on
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            elif(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):         #check if vehicle armed
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                
                last_req = rospy.Time.now()
            
            elif  self.isclose(self.goalpose.pose.position.x,self.nowPose.pose.pose.position.x,0.1)&\
                self.isclose(self.goalpose.pose.position.y,self.nowPose.pose.pose.position.y,0.1)&\
                self.isclose(self.goalpose.pose.position.z,self.nowPose.pose.pose.position.z,0.1):              #check if position reached goal position for new goal
                self.current_coord = self.get_new_coord(self.current_coord)
                self.goalpose.pose.position.x = self.current_coord[0]
                self.goalpose.pose.position.y = self.current_coord[1]
                self.goalpose.pose.position.z = self.desired_height
                print("reached")
                print (self.current_coord)
                print (self.coordinate_count)

            else :                                                                                              #keeps publishing goal position for FC to follow
                self.goalpose.pose.position.x = self.current_coord[0]
                self.goalpose.pose.position.y = self.current_coord[1]
                self.goalpose.pose.position.z = self.desired_height
                self.local_pos_pub.publish(self.goalpose)

        rate.sleep()
           
        
if __name__ == '__main__':
    rospy.init_node("offb_node_py")
    drone = Drone(1,4,8,10,6)
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