#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class Drone:
    def __init__(self,desired_height): 
        self.current_state = State()
        self.desired_height = desired_height
        self.current_coord = (0,0) #initialize 1st position
        self.offset_for_newcircle = 0.1 #starting and end loop for circles are same so it iterates back to for the starting circling of infinity.
        self.coordinate_count = 0
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, callback = self.local_position_callback, queue_size=10)
        self.sub = rospy.Subscriber('/laser/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/laser/revised_scan', LaserScan, queue_size = 10)
        self.scann = LaserScan()
        self.landpos = [(0.5,1.5)]
        self.fly()

    def path(self):
        return [(0,0),(8,0),(5,5),(0,5),(0,0)]


    def get_latest_path(self):
        global num_of_points_in_path 
        num_of_points_in_path = len(self.path())
        return self.path()
    
    def isclose(self,a,b,abs_tol):
        return(abs(a-b) <= abs_tol)

    def landing(self):
        self.offb_set_mode.custom_mode = 'AUTO.LAND'
        self.goalpose.pose.position.z = 0

    def get_new_coord(self,current_coord):
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


    def callback(self,msg):
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
        self.scann.ranges = msg.ranges[::5]
        coll = []
        for i in range(len(self.scann.ranges)):
            if 1 <= (self.scann.ranges[i]) <= 5:      #filter bad data
                coll.append((i,self.scann.ranges[i]))
        print(coll)
        '''
        function to conjoin last and first chunking to get point nearest obstacle
        '''

        #if (coll[0]-coll[-1])
        #print ((coll[0]+coll[-1])/2)
        #self.scann.intensities = msg.intensities[0:40]
        self.pub.publish(self.scann)


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
                    print("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            elif(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):         #check if vehicle armed
                if(arming_client.call(arm_cmd).success == True):
                    print("Vehicle armed")
                
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
