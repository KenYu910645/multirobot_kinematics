#!/usr/bin/env python
import rospy 
import sys
import time
from math import sin, cos , pi, atan2 ,acos,asin, sqrt
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped# Global path
from tf import transformations
import tf
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
# These are for testing performance 
import cProfile
import re
import pstats # for sorting result 
import random
import numpy as np
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

DT = 0.5 # sec
BIG_NUM = 100 # A very big number for infinity line drawing 
NUM_LINE_SEG =  30 # For arc stepping 
INF_SMALL = 0.00000001
EQUALITY_ERROR = 0.000001
L = 4
#--- init value -----# 
INIT_VC = 5
INIT_WC = 5
SLIDER_GAIN = 1
#---- Global variable ----# 
server = None # For interative markers
marker_sphere = MarkerArray()
marker_line   = MarkerArray()
marker_text   = MarkerArray()

def normalize_angle(angle):
    '''
    [-pi, pi]
    '''
    sign = None 
    if angle >= 0:
        sign = 1
    else: 
        sign = -1 

    ans = angle % (2* pi * sign)

    if ans < -pi: # [-2pi, -pi]
        ans += 2*pi
    elif ans > pi: # [pi, 2pi]
        ans -= 2*pi
    return ans
    
class Car():
    def __init__(self, id, init_kine): # unique id for every robot 
        self.id = id 
        (self.x,self.y,self.theta,self.v,self.w) = init_kine
        # --Center of rotation-- # 
        self.x_c = 0
        self.y_c = 0
        self.r = 0
        # --Final position-- # 
        self.x_p = 0
        self.y_p = 0
        self.theta_p = 0
        self.kinematic_current = PoseStamped()
        self.kinematic_result  = PoseStamped()
        #-- Publish ---#
        self.is_need_pub = True
        
    def cal_FK(self):
        '''
        calculate forward kinematics
        Input : 
            self.x
            self.y
            self.theta
            self.v
            self.w
        Output: 
            self.x_p
            self.y_p
            self.theta_p
            #
            self.x_c
            self.y_c
            self.r
        '''
        (self.x_p, self.y_p, self.theta_p, self.x_c, self.y_c ,self.r) = self.cal_FK_passing_paramters(self.x, self.y, self.theta, self.v, self.w)

    def cal_FK_passing_paramters(self, x,y,theta,v,w):
        '''
        output (x_p,y_p,theta_p,x_c,y_c)
        '''
        try: 
            r = v/w # divide by zero
        except ZeroDivisionError:
            # w = INF_SMALL
            # r = v/w
            r = float('inf')
        # --- rotation center ----# 
        x_c = x - r*sin(theta)
        y_c = y + r*cos(theta)
        # --- result -----# 
        x_p = x_c + r*sin(theta + w*DT)
        y_p = y_c - r*cos(theta + w*DT)
        theta_p = normalize_angle (theta + w*DT)
        return (x_p,y_p,theta_p,x_c,y_c,r)

    def cal_IK(self):
        '''
        calculate inverse kinematics
        This should be only called by car_1, car_2
        Input : 
            self.x
            self.y
            self.theta
            self.x_p
            self.y_p
        Output:
            self.theta_p
            self.v
            self.w
            self.r
        '''
        A = 2*(self.x_p - self.x)
        B = 2*(self.y_p - self.y)
        C = self.x_p**2 + self.y_p**2 - self.x**2 - self.y**2
        D = cos(self.theta)
        E = sin(self.theta)
        F = cos(self.theta) * self.x + sin(self.theta) * self.y

        LHS = np.array([[A,B],[D,E]])
        RHS = np.array([C,F])
        try: 
            (self.x_c, self.y_c) = np.linalg.solve(LHS,RHS)
        except np.linalg.linalg.LinAlgError:
            print ("NO SOLUTION!!!!!!!!!!!!!!!!!")
        
        self.r = sqrt( (self.x - self.x_c)**2 + (self.y - self.y_c)**2 )
        
        self.theta_p = normalize_angle(atan2(self.x_c - self.x_p , self.y_p - self.y_c))
        #---- theta_p has a constraint that direction must be the same as the votex -----# 
        v_s = np.cross( [ self.x - self.x_c , self.y - self.y_c ,0 ]   , [ cos(self.theta) , sin(self.theta),0] )
        v_e = np.cross( [ self.x_p - self.x_c , self.y_p - self.y_c,0] , [ cos(self.theta_p) , sin(self.theta_p),0] )
        if v_s[2] * v_e [2] < 0: # different sign 
            self.theta_p = normalize_angle(self.theta_p + pi) # Switch the direction 
        
        #---- Go forward of Go backward ? ----#
        '''
        dtheta = normalize_angle(self.theta_p - self.theta)
        if   dtheta > pi :
            self.w = (self.theta_p - self.theta - 2*pi) / DT
        elif dtheta < -pi :
            self.w = (self.theta_p - self.theta + 2*pi) / DT
        '''
        self.w = (self.theta_p - self.theta) / DT

        if self.w != 0:
            self.v = self.w * self.r 
        else : 
            self.v = (self.x_p - self.x) / DT

        if self.id == 0:
            print ("self.r: " + str(self.r))
            print ("self.w: " + str(self.w))
            print ("self.v: " + str(self.v))

    
    def update_markers(self):
        
        # ----- update kinematics result(Final Pose) ------# 
        self.kinematic_result.header.frame_id = "base_link"
        self.kinematic_result.header.stamp = rospy.get_rostime()
        self.kinematic_result.pose.position.x = self.x_p
        self.kinematic_result.pose.position.y = self.y_p
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta_p)
        self.kinematic_result.pose.orientation.x = q[0]
        self.kinematic_result.pose.orientation.y = q[1]
        self.kinematic_result.pose.orientation.z = q[2]
        self.kinematic_result.pose.orientation.w = q[3]
        
        #------ update kinematics current (init pose) ---# 
        self.kinematic_current.header.frame_id = "base_link"
        self.kinematic_current.header.stamp = rospy.get_rostime()
        self.kinematic_current.pose.position.x = self.x
        self.kinematic_current.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.kinematic_current.pose.orientation.x = q[0]
        self.kinematic_current.pose.orientation.y = q[1]
        self.kinematic_current.pose.orientation.z = q[2]
        self.kinematic_current.pose.orientation.w = q[3]

        # ---- update rotation center -----# 
        modify_sphere((self.x_c,self.y_c), self.id)
        # ---- update path arc -----#
        
        points = []
        if abs(self.w) <= INF_SMALL: # draw a straight line
            points.append( (self.x   , self.y) )
            points.append( (self.x_p , self.y_p) )
        else: 
            theta_start = atan2( (self.y   - self.y_c) ,(self.x   - self.x_c) )
            theta_end   = atan2( (self.y_p - self.y_c) ,(self.x_p - self.x_c) )
            for i in range (NUM_LINE_SEG + 1):
                t = (DT/NUM_LINE_SEG)*i
                point = (self.x_c + self.r*cos(theta_start + self.w*t), self.y_c + self.r*sin(theta_start + self.w*t))
                points.append(point)
            
        modify_line(points, self.id)
        
        # --Allow main loop to publish markers-- # 
        self.is_need_pub = True
    
def cal_small_car_position(pose):
    '''
    This should be only called by car_big
    Input : 
        pose = (x,y,theta)
    Output: 
        [(x1,y1),(x2,y2)]
    '''
    x1 = pose[0] + (L/2)*cos(pose[2])
    y1 = pose[1] + (L/2)*sin(pose[2])
    x2 = pose[0] - (L/2)*cos(pose[2])
    y2 = pose[1] - (L/2)*sin(pose[2])
    return ((x1,y1),(x2,y2))

def marker_feedback_CB(data):
    if data.marker_name[:3] == "car":
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        if data.marker_name == "car_1":
            if data.pose.position.x != 0 or data.pose.position.y != 0: 
                data.pose.position.x = 0
                data.pose.position.y = 0
                car_1.x = 0
                car_1.y = 0
                car_1.theta = normalize_angle (euler[2])
                server.setPose( data.marker_name, data.pose )
                server.applyChanges()
            else: 
                car_1.x     = data.pose.position.x
                car_1.y     = data.pose.position.y
                car_1.theta = normalize_angle (euler[2])

        elif data.marker_name == "car_2":
            yaw = atan2(data.pose.position.y, data.pose.position.x)
            if data.pose.position.x != L * cos(yaw) or data.pose.position.y != L * sin(yaw):
                data.pose.position.x = L * cos(yaw)
                data.pose.position.y = L * sin(yaw)
                car_2.x = L * cos(yaw)
                car_2.y = L * sin(yaw)
                car_2.theta = normalize_angle (euler[2])
                server.setPose( data.marker_name, data.pose )
                server.applyChanges()
            else: 
                car_2.x     = data.pose.position.x
                car_2.y     = data.pose.position.y
                car_2.theta = normalize_angle (euler[2])
    #---- Sliders -----# 
    elif data.marker_name == "Vc":
        car_big.v = data.pose.position.y * SLIDER_GAIN
        marker_text.markers[0].text = str(round(car_big.v ,2))
        marker_text.markers[0].pose.position.y = data.pose.position.y+1

    elif data.marker_name == "Wc":
        car_big.w = data.pose.position.y * SLIDER_GAIN
        marker_text.markers[1].text = str(round(car_big.w ,2))
        marker_text.markers[1].pose.position.y = data.pose.position.y+1
    
    #--- update car_big ------# 
    car_big.x = (car_1.x+car_2.x)/2
    car_big.y = (car_1.y+car_2.y)/2
    car_big.theta = normalize_angle (atan2(car_2.y-car_1.y, car_2.x-car_1.x))
    car_big.cal_FK()

    (p1,p2) = cal_small_car_position((car_big.x_p,car_big.y_p,car_big.theta_p))
    #--- update car_1,car_2 ----# 
    (car_1.x_p,car_1.y_p) = p2
    (car_2.x_p,car_2.y_p) = p1
    #--- inversely calculate v,w ---#
    car_1.cal_IK()
    car_2.cal_IK()

    #--- Update Textes ----# 
    marker_text.markers[2].text = "V_car_1 : " + str(round(car_1.v ,2))
    marker_text.markers[3].text = "W_car_1 : " + str(round(car_1.w ,2))
    marker_text.markers[4].text = "V_car_2 : " + str(round(car_2.v ,2))
    marker_text.markers[5].text = "W_car_2 : " + str(round(car_2.w ,2))
    marker_text.markers[6].text = "Theta_car_1 : "   + str(round(car_1.theta   ,2))
    marker_text.markers[7].text = "Theta_P_car_1 : " + str(round(car_1.theta_p ,2))
    marker_text.markers[8].text = "Theta_car_2 : "   + str(round(car_2.theta   ,2))
    marker_text.markers[9].text = "Theta_P_car_2 : " + str(round(car_2.theta_p ,2))
    #--- Update markers ----# 
    car_1.update_markers()
    car_2.update_markers()
    car_big.update_markers()

def set_sphere(point , RGB = None  , size = 0.05, id = 0):
    '''
    Set Point at MarkArray 
    Input : 
        point - (x,y)
        RGB - (r,g,b)
    '''
    global marker_sphere
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    (marker.pose.position.x , marker.pose.position.y) = point
    marker_sphere.markers.append(marker)

def set_line( points , RGB = None , size = 0.2, id = 0):
    '''
    Set line at MarkArray
    Input : 
        points = [p1,p2....] 
    '''
    global marker_line

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    for i in points : 
        p = Point()
        p.x = i[0]
        p.y = i[1]
        marker.points.append(p)
    marker_line.markers.append(marker)

def set_text(point,text , RGB = None  , size = 0.2, id = 0):
    '''
    Set Point at MarkArray 
    Input : 
        point - (x,y) or idx 
        RGB - (r,g,b)
    '''
    global marker_text
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    marker.text = text 
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    (marker.pose.position.x , marker.pose.position.y) = point
    marker_text.markers.append(marker)

def make6DofMarker( fixed, interaction_mode, position, show_6dof = False, name="marker"):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = ""
    int_marker.description = ""

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    int_marker.name = name 
    int_marker.description = name 
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    server.insert(int_marker, processFeedback)
    # menu_handler.apply( server, int_marker.name )

def makeYaxisMarker(position, name ):
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.pose.position = position
    int_marker.scale = 1

    q = tf.transformations.quaternion_from_euler(0, 0, pi/2)
    int_marker.pose.orientation.x = q[0]
    int_marker.pose.orientation.y = q[1]
    int_marker.pose.orientation.z = q[2]
    int_marker.pose.orientation.w = q[3]

    int_marker.pose.orientation.x
    int_marker.header.frame_id = "base_link"
    int_marker.name = name
    int_marker.description = name

    # create a grey box marker
    box_marker = makeBox(int_marker)

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_y"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control);

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

def car_1_constriant( feedback ):
    pose = feedback.pose
    if pose.position.x != 0 or pose.position.y != 0: 
        pose.position.x = 0
        pose.position.y = 0
        car_1.x = 0
        car_1.y = 0
        car_1.cal_IK()  
        car_1.update_markers()
        server.setPose( feedback.marker_name, pose )
        server.applyChanges()
    

def car_2_constriant( feedback ):
    pose = feedback.pose
    yaw = atan2(pose.position.y, pose.position.x)
    if pose.position.x != L * cos(yaw) or pose.position.y != L * sin(yaw):
        pose.position.x = L * cos(yaw)
        pose.position.y = L * sin(yaw)
        car_2.x = L * cos(yaw)
        car_2.y = L * sin(yaw)
        car_2.cal_FK() 
        car_2.update_markers()
        server.setPose( feedback.marker_name, pose )
        server.applyChanges()
    

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        # rospy.loginfo( s + ": button click" + mp + "." )
        pass 
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        # rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        pass 
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        # rospy.loginfo( s + ": pose changed")
        pass 

#--- Init cars -----# 
#           id ,x,y,theta,v,w
car_1   = Car(0, (0,0,0,0,0))
car_2   = Car(1, (L,0,0,0,0))
car_big = Car(2, (L/2,0,0,INIT_VC,INIT_WC))
 # v1,w1,v2,w2 need to be init
def modify_line(points, id):
    global marker_line
    ans=[]
    for i in points : 
        ans.append(Point(i[0],i[1],0))
    marker_line.markers[id].points = ans

def modify_sphere(point, id):
    '''
    Input:
        point: (x,y)
    '''
    global marker_sphere
    marker_sphere.markers[id].pose.position.x = point[0]
    marker_sphere.markers[id].pose.position.y = point[1]

def main(args):
    global server
    #----- Init node ------# 
    rospy.init_node('kinematics', anonymous=True)
    #----- Subscribers ------# 
    rospy.Subscriber('/basic_controls/feedback', InteractiveMarkerFeedback, marker_feedback_CB)
    #----- Publisher -------# 
    pub_car_1_result    = rospy.Publisher('car_1_result'  , PoseStamped ,queue_size = 10,  latch=False)
    pub_car_2_result    = rospy.Publisher('car_2_result'  , PoseStamped ,queue_size = 10,  latch=False)
    pub_car_big_result  = rospy.Publisher('car_big_result', PoseStamped ,queue_size = 10,  latch=False)
    pub_car_1_current    = rospy.Publisher('car_1_current'  , PoseStamped ,queue_size = 10,  latch=False)
    pub_car_2_current    = rospy.Publisher('car_2_current'  , PoseStamped ,queue_size = 10,  latch=False)
    pub_car_big_current  = rospy.Publisher('car_big_current', PoseStamped ,queue_size = 10,  latch=False)
    #--   publish marker --# 
    pub_marker_sphere     = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=True)
    pub_marker_line       = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=True)
    pub_marker_text       = rospy.Publisher('marker_text'  , MarkerArray,queue_size = 1,latch=True)
    #------- Interactive Markers ---------# 
    server = InteractiveMarkerServer("basic_controls")

    #------- Markers: Two cars -----------# 
    position = Point(0,0,0)
    make6DofMarker( False , InteractiveMarkerControl.MOVE_ROTATE_3D, position, True , "car_1")
    # server.setCallback("car_1", car_1_constriant, InteractiveMarkerFeedback.POSE_UPDATE )
    position = Point( L,0,0)
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, True , "car_2")
    # server.setCallback("car_2", car_2_constriant, InteractiveMarkerFeedback.POSE_UPDATE )
    #------- Markers: Two sliders -----------# 
    position = Point(-3,  INIT_VC/SLIDER_GAIN,0)
    makeYaxisMarker(position, "Vc")
    position = Point(-3.5,INIT_WC/SLIDER_GAIN,0)
    makeYaxisMarker(position, "Wc")
    
    r = rospy.Rate(30) #call at 30HZ
    # --- init publish markers ---# 

    p = cal_small_car_position((car_big.x_p,car_big.y_p, car_big.theta_p)) # ((x1,y1),(x2,y2))

    (car_1.x_p,car_2.y_p) = p[0]
    car_1.cal_IK()

    (car_2.x_p,car_2.y_p) = p[1]
    car_2.cal_IK()
    
    car_1.cal_FK()
    car_2.cal_FK()
    car_big.cal_FK()


    # --- init text markers ---# 
    set_text((-3  ,INIT_VC+1) , str(round(car_big.v ,2)) , (255,255,255) , 0.3, 0) # Vc slider text
    set_text((-3.5,INIT_VC+1) , str(round(car_big.w ,2)) , (255,255,255) , 0.3, 1) # Wc slider text
    set_text((0,-1)   , "V_car_1 : " + str(round(car_1.v ,2)) , (255,255,255) , 0.3, 2)
    set_text((0,-1.5) , "W_car_1 : " + str(round(car_1.w ,2)) , (255,255,255) , 0.3, 3)
    set_text((0,-2)   , "V_car_2 : " + str(round(car_2.v ,2)) , (255,255,255) , 0.3, 4)
    set_text((0,-2.5) , "W_car_2 : " + str(round(car_2.w ,2)) , (255,255,255) , 0.3, 5)
    set_text((-3,0)    , "Theta_car_1 : " + str(round(car_1.theta ,2)) , (255,255,255) , 0.3, 6)
    set_text((-3,-0.5) , "Theta_P_car_1 : " + str(round(car_1.theta_p ,2)) , (255,255,255) , 0.3, 7)
    set_text((-3,-1)   , "Theta_car_2 : " + str(round(car_2.theta ,2)) , (255,255,255) , 0.3, 8)
    set_text((-3,-1.5) , "Theta_P_car_2 : " + str(round(car_2.theta_p ,2)) , (255,255,255) , 0.3, 9)
    #---- init lines ------# Yellow line for arcs 
    set_line([], (255,255,0), 0.02,0) # car_1 
    set_line([], (255,255,0), 0.02,1) # car_2
    set_line([], (255,255,0), 0.02,2) # car_big
    p = cal_small_car_position((car_big.x, car_big.y, car_big.theta))
    set_line([p[0],p[1]],(150,0,0),0.1,3) # REd line 
    p = cal_small_car_position((car_big.x_p, car_big.y_p, car_big.theta_p))
    set_line([p[0],p[1]],(255,0,0),0.1,4) # Red line 
    #---- init spheres -----# 
    set_sphere((car_1.x_c,car_1.y_c),     (0,255,255) , 0.2, 0)
    set_sphere((car_2.x_c,car_2.y_c),     (0,255,255) , 0.2, 1)
    set_sphere((car_big.x_c,car_big.y_c), (0,255,255) , 0.2, 2)

    car_1.update_markers()
    car_2.update_markers()
    car_big.update_markers()

    server.applyChanges()

    while (not rospy.is_shutdown()):
        if  car_1.is_need_pub or car_2.is_need_pub or car_big.is_need_pub:
            #---- update car big pose ----# 
            modify_line(cal_small_car_position((car_big.x  , car_big.y  , car_big.theta))   ,3)
            modify_line(cal_small_car_position((car_big.x_p, car_big.y_p, car_big.theta_p)) ,4)
            #---- update car_1 ----# 
            pub_car_1_result.publish(car_1.kinematic_result)
            pub_car_1_current.publish(car_1.kinematic_current)
            #---- update car_2 -----# 
            pub_car_2_result.publish(car_2.kinematic_result)
            pub_car_2_current.publish(car_2.kinematic_current)
            #----- update car_big ----# 
            pub_car_big_result.publish(car_big.kinematic_result)
            pub_car_big_current.publish(car_big.kinematic_current)

            #---- update Textes -----# 
            pub_marker_text.publish(marker_text)
            #---- update center of rotations -----# 
            pub_marker_sphere.publish(marker_sphere)
            #---- update lines -----# 
            pub_marker_line.publish(marker_line)

            #--- Reset flags ------# 
            car_1.is_need_pub  = False
            car_2.is_need_pub  = False
            car_big.is_need_pub = False
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass