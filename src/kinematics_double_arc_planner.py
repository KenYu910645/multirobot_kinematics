#!/usr/bin/env python
import rospy 
import sys
import time
from math import sin, cos , pi, atan2 ,acos,asin, sqrt
import random
import numpy as np
# ROS
from tf import transformations
import tf
# msg type
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
# Custom import
from lucky_utility.ros.rospy_utility import Marker_Manager, normalize_angle

DT = 0.5 # sec
BIG_NUM = 100 # A very big number for infinity line drawing 
NUM_LINE_SEG =  30 # For arc stepping 
INF_SMALL = 0.00000001
EQUALITY_ERROR = 0.000001
L = 4
#--- init value -----# 
INIT_VC = 3
INIT_WC = 0.6
SLIDER_GAIN = 1
INIT_CAR1_THETA = pi/3
INIT_CAR2_THETA = pi/4
INIT_CARBIG_THETA = pi/4

############################
###   Utility Function   ###
############################

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

class Car():
    def __init__(self, id, init_kine, marker_center_name, marker_arc_name): # unique id for every robot 
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
        # marker name
        self.marker_center_name = marker_center_name
        self.marker_arc_name = marker_arc_name
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
        (self.x_p,
         self.y_p,
         self.theta_p,
         self.x_c, 
         self.y_c,
         self.r) = self.cal_FK_passing_paramters(self.x, 
                                                 self.y, 
                                                 self.theta, 
                                                 self.v, 
                                                 self.w)

    def cal_FK_passing_paramters(self, x,y,theta,v,w):
        '''
        output (x_p,y_p,theta_p,x_c,y_c)
        '''
        try: 
            r = v/w # divide by zero
        except ZeroDivisionError:
            # w = INF_SMALL
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
        self.w = (self.theta_p - self.theta) / DT
        dtheta = self.theta_p - self.theta
        if   dtheta > pi :
            self.w = (self.theta_p - self.theta - 2*pi) / DT
        elif dtheta < -pi :
            self.w = (self.theta_p - self.theta + 2*pi) / DT
        
        if self.w != 0:
            self.v = self.w * self.r 
        else : 
            self.v = (self.x_p - self.x) / DT

    def update_markers(self):
        #------ update kinematics current (init pose) ---# 
        self.kinematic_current.header.frame_id = "base_link"
        self.kinematic_current.header.stamp = rospy.get_rostime()
        self.kinematic_current.pose.position = Point(self.x, self.y, 0)

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.kinematic_current.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        # ----- update kinematics result(Final Pose) ------# 
        self.kinematic_result.header.frame_id = "base_link"
        self.kinematic_result.header.stamp = rospy.get_rostime()
        self.kinematic_result.pose.position = Point(self.x_p, self.y_p, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta_p)
        self.kinematic_result.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        # ---- update rotation center -----#
        MARKER_MANAGER.update_marker(self.marker_center_name, (self.x_c,self.y_c))
        # modify_sphere((self.x_c,self.y_c), self.id)
        # ---- update path arc -----#
        theta_start = atan2( (self.y   - self.y_c) ,(self.x   - self.x_c) )
        # TODO clock-wise  , couter clock-wise
        MARKER_MANAGER.update_marker(self.marker_arc_name,
                                     (),
                                     radius = self.r,
                                     angle_range = (theta_start, theta_start + self.w*DT))
        # points = []
        # if abs(self.w) <= INF_SMALL: # draw a straight line
        #     points.append( (self.x   , self.y) )
        #     points.append( (self.x_p , self.y_p) )
        # else: 

        #     theta_start = atan2( (self.y   - self.y_c) ,(self.x   - self.x_c) )         
        #     for i in range (NUM_LINE_SEG + 1):
        #         t = (DT/NUM_LINE_SEG)*i
        #         point = (self.x_c + abs(self.r)*cos(theta_start + self.w*t), self.y_c + abs(self.r)*sin(theta_start + self.w*t))
        #         points.append(point)
        # modify_line(points, self.id)
        
        # --Allow main loop to publish markers-- # 
        self.is_need_pub = True

    def feedback_cb(self, data):
        pass
###########################################
###  Interactive Marker Modification    ###
###########################################
def make6DofMarker(name, marker_type, frame_id, scale, RGB, feedback_cb, 
                   fixed, interaction_mode, pose, show_6dof = True):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose = pose
    int_marker.scale = scale

    # int_marker.name = ""
    # int_marker.description = ""
    int_marker.name = name
    int_marker.description = name

    # insert a box
    # makeBoxControl(int_marker)

    marker = Marker()
    marker.type = marker_type
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = RGB[0]/255.0
    marker.color.g = RGB[1]/255.0
    marker.color.b = RGB[2]/255.0
    marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( marker )
    int_marker.controls.append( control )
    int_marker.controls[0].interaction_mode = interaction_mode

    # if fixed:
    #     int_marker.name += "_fixed"
    #     int_marker.description += "\n(fixed orientation)"

    # if interaction_mode != InteractiveMarkerControl.NONE:
    #     control_modes_dict = { 
    #                       InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
    #                       InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
    #                       InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
    #     int_marker.name += "_" + control_modes_dict[interaction_mode]
    #     int_marker.description = "3D Control"
    #     if show_6dof: 
    #       int_marker.description += " + 6-DOF controls"
    #     int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    # int_marker.name = name 
    # int_marker.description = name 
    
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
    SERVER.insert(int_marker, marker_feedback_CB)

def makeYaxisMarker(position, name ):
    # create an interactive marker for our SERVER
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
    SERVER.insert(int_marker, marker_feedback_CB)

#############################
###  Marker CB Function   ###
#############################
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
                SERVER.setPose( data.marker_name, data.pose )
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
                SERVER.setPose( data.marker_name, data.pose )
            else: 
                car_2.x     = data.pose.position.x
                car_2.y     = data.pose.position.y
                car_2.theta = normalize_angle (euler[2])

    #--- Update Textes ----# 
    # marker_text.markers[2].text = "V_car_1 : " + str(round(car_1.v ,2))
    # marker_text.markers[3].text = "W_car_1 : " + str(round(car_1.w ,2))
    # marker_text.markers[4].text = "V_car_2 : " + str(round(car_2.v ,2))
    # marker_text.markers[5].text = "W_car_2 : " + str(round(car_2.w ,2))
    #--- Update markers ----# 
    car_1.update_markers()
    car_2.update_markers()
    #--- Server change ----# 
    SERVER.applyChanges()

if __name__ == '__main__':
    #----- Init node ------# 
    rospy.init_node('kinematics_double_arc', anonymous=True)
    #----- Publisher -------# 
    pub_car_1_result     = rospy.Publisher('car_1_result'  , PoseStamped ,queue_size = 10,  latch=True)
    pub_car_2_result     = rospy.Publisher('car_2_result'  , PoseStamped ,queue_size = 10,  latch=True)
    pub_car_1_current    = rospy.Publisher('car_1_current'  , PoseStamped ,queue_size = 10,  latch=True)
    pub_car_2_current    = rospy.Publisher('car_2_current'  , PoseStamped ,queue_size = 10,  latch=True)
    #------- Interactive Markers ---------# 
    SERVER = InteractiveMarkerServer("basic_controls")
    MARKER_MANAGER = Marker_Manager("markers")
   
    #--- Init cars -----# 
    #           id ,x,y,theta,v,w
    car_1   = Car(0, (0,0,INIT_CAR1_THETA,0,0), "arc_center_start", "r_start")
    car_2   = Car(1, (0,0,INIT_CAR2_THETA,0,0), "arc_center_end", "r_end")
    (p1, p2) = cal_small_car_position( ( (L/2)*cos(INIT_CARBIG_THETA) , (L/2)*sin(INIT_CARBIG_THETA)  , INIT_CARBIG_THETA ) )
    car_1.x = p2[0]
    car_1.y = p2[1]
    car_2.x = p1[0]
    car_2.y = p1[1]

    #------- Markers: Two cars -----------# 
    q = tf.transformations.quaternion_from_euler(0, 0, car_1.theta)
    pose = Pose(Point(car_1.x,car_1.y,0), Quaternion(q[0],q[1],q[2],q[3])) # 0 degree
    make6DofMarker("car_1", 0, "base_link", 0.2, (255,0,0), car_1.feedback_cb,
                   False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True)
    q = tf.transformations.quaternion_from_euler(0, 0, car_2.theta)
    pose = Pose(Point(car_2.x,car_2.y,0), Quaternion(q[0],q[1],q[2],q[3]))# 45 degree
    # make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True , "car_2")
    make6DofMarker("car_2", 0, "base_link", 0.2, (255,0,0), car_2.feedback_cb,
                   False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True)
    # --- init text markers ---# 
    # set_text((0,-1)   , "V_car_1 : " + str(round(car_1.v ,2)) , (255,255,255) , 0.3, 2)
    # set_text((0,-1.5) , "W_car_1 : " + str(round(car_1.w ,2)) , (255,255,255) , 0.3, 3)
    # set_text((0,-2)   , "V_car_2 : " + str(round(car_2.v ,2)) , (255,255,255) , 0.3, 4)
    # set_text((0,-2.5) , "W_car_2 : " + str(round(car_2.w ,2)) , (255,255,255) , 0.3, 5)

    #---- init lines ------# Yellow line for arcs
    MARKER_MANAGER.register_marker("r_start", 4 , "base_link", (255,255,0), 0.02)
    MARKER_MANAGER.register_marker("r_end"  , 4 , "base_link", (255,255,0), 0.02)
    # Center of rotation
    MARKER_MANAGER.register_marker("arc_center_start", 2 ,"base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("arc_center_end", 2 ,"base_link", (255,255,255), 0.2)

    #---- Init Markers ----# 
    init = InteractiveMarkerFeedback()
    init.marker_name = ""
    init.header.frame_id = "base_link"
    marker_feedback_CB(init)

    r = rospy.Rate(30) #call at 30HZ
    while (not rospy.is_shutdown()):
        if  car_1.is_need_pub or car_2.is_need_pub:
            #---- update car_1 ----# 
            pub_car_1_result.publish(car_1.kinematic_result)
            pub_car_1_current.publish(car_1.kinematic_current)
            #---- update car_2 -----# 
            pub_car_2_result.publish(car_2.kinematic_result)
            pub_car_2_current.publish(car_2.kinematic_current)
            #---- update Textes -----# 
            # pub_marker_text.publish(marker_text)
            MARKER_MANAGER.publish()
            #--- Reset flags ------# 
            car_1.is_need_pub  = False
            car_2.is_need_pub  = False
        r.sleep()

