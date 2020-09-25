#!/usr/bin/env python
import rospy 
import sys
import time
from math import sin, cos , pi, atan2 ,acos, asin, sqrt 
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
from lucky_utility.ros.rospy_utility import Marker_Manager, normalize_angle, cal_ang_distance, sign

DT = 0.5 # sec
BIG_NUM = 100 # A very big number for infinity line drawing 
NUM_LINE_SEG =  30 # For arc stepping 
INF_SMALL = 0.00000001
EQUALITY_ERROR = 0.000001
L = 4
#--- init value -----# 
INIT_CAR1_THETA = pi/3
INIT_CAR2_THETA = pi/4

def make6DofMarker(name, marker_type, frame_id, scale, RGB, feedback_cb, 
                   fixed, interaction_mode, pose, show_6dof = True):
    '''
    scale.x is the shaft diameter, 
    and scale.y is the head diameter. 
    If scale.z is not zero, it specifies the head length. 
    '''
    # Init interactive markers
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose = pose
    int_marker.scale = scale
    int_marker.name = name
    int_marker.description = name

    # Init visualization markers
    marker = Marker()
    marker.type = marker_type
    if marker_type == 0: # ARROW
        marker.scale.x = scale * 1.0
        marker.scale.y = scale * 0.1
        marker.scale.z = scale * 0.1
    else:
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
    marker.color.r = RGB[0]/255.0
    marker.color.g = RGB[1]/255.0
    marker.color.b = RGB[2]/255.0
    marker.color.a = 1.0

    # Set controller
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( marker )
    int_marker.controls.append(control)
    int_marker.controls[0].interaction_mode = interaction_mode

    # Flags 
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
    
    # Insert new interative marker to server
    SERVER.insert(int_marker, feedback_cb)

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

def marker_feedback_CB(data):
    pass 

def car_m_feedback_cb(data):
    '''
    '''
    global CAR_MID_XYT
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    (_,_,yaw) = transformations.euler_from_quaternion(quaternion)
    CAR_MID_XYT = (data.pose.position.x, data.pose.position.y, yaw)

def cal_double_arc(start_xyt, end_xyt):
    A =  sin(start_xyt[2])
    B = -sin(end_xyt[2])
    C = start_xyt[0] - end_xyt[0]
    D = -cos(start_xyt[2])
    E =  cos(end_xyt[2])
    F = start_xyt[1] - end_xyt[1]

    LHS = np.array([[A,B],[D,E]])
    RHS = np.array([C,F])
    try:
        (r1, r2) = np.linalg.solve(LHS,RHS)
    except np.linalg.linalg.LinAlgError:
        print ("NO SOLUTION!!!!!!!!!!!!!!!!!")
    else:
        print ("r1 = " + str(r1))
        print ("r2 = " + str(r2))

class Car():
    def __init__(self, name, init_kine, marker_center_name, marker_arc_name):
        (self.x,self.y,self.theta,self.v,self.w) = init_kine
        self.name = name
        # --Center of rotation-- # 
        self.x_c = 0
        self.y_c = 0
        self.dir_center = "RHS" # 'LHS'
        self.r = 0
        # --Final position-- # 
        self.x_p = 0
        self.y_p = 0
        self.theta_p = 0
        self.kinematic_result  = PoseStamped()
        # marker name
        self.marker_center_name = marker_center_name
        self.marker_arc_name = marker_arc_name
        
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
        
        # Judge center of rotation is at RHS or LHS
        # if self.name == "start":
        dtheta = self.theta_p - self.theta
        if v_s[2] > 0 :
            self.dir_center = "LHS"
            # LHS
            if dtheta > 0.0:
                # print ("Forward")
                self.v = 1.0
            else:
                # print("backward")
                self.v = -1.0
        else:
            self.dir_center = "RHS"
            # RHS
            if dtheta < 0.0:
                # print ("Forward")
                self.v = 1.0
            else:
                # print("backward")
                self.v = -1.0
        
        #---- Go forward of Go backward ? ----#
        self.w = (self.theta_p - self.theta) / DT
        dtheta = self.theta_p - self.theta
        if   dtheta > pi :
            self.w = (self.theta_p - self.theta - 2*pi) / DT
        elif dtheta < -pi :
            self.w = (self.theta_p - self.theta + 2*pi) / DT
        
        if self.w != 0:
            self.v = abs(self.w * self.r) * sign(self.v)
        else:
            self.v = (self.x_p - self.x) / DT

    def update_markers(self):
        # ----- update kinematics result(Final Pose) ------# 
        self.kinematic_result.header.frame_id = "base_link"
        self.kinematic_result.header.stamp = rospy.get_rostime()
        self.kinematic_result.pose.position = Point(self.x_p, self.y_p, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta_p)
        self.kinematic_result.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        # ---- update rotation center -----#
        # MARKER_MANAGER.update_marker(self.marker_center_name, (self.x_c,self.y_c))

        # ---- update text --------# 
        if self.name == "start":
            MARKER_MANAGER.update_marker("v1", (2,0), text = "v1=" + str(round(self.v,3)))
            MARKER_MANAGER.update_marker("w1", (2,-0.3), text = "w1=" + str(round(self.w,3)))
            MARKER_MANAGER.update_marker("r1", (2,-0.6), text = "r1=" + str(round(self.r,3)))
            MARKER_MANAGER.update_marker("theta_p1", (2,-0.9), text = "dtheta1=" + str(round(self.theta_p - self.theta,3)))
            MARKER_MANAGER.update_marker("length1", (2,-1.2), text = "length1=" + str(round((self.theta_p - self.theta)*self.r,3)))
        if self.name == "end":
            MARKER_MANAGER.update_marker("v2", (4,0), text = "v2=" + str(round(self.v,3)))
            MARKER_MANAGER.update_marker("w2", (4,-0.3), text = "w2=" + str(round(self.w,3)))
            MARKER_MANAGER.update_marker("r2", (4,-0.6), text = "r2=" + str(round(self.r,3)))
            MARKER_MANAGER.update_marker("theta_p2", (4,-0.9), text = "dtheta2=" + str(round(self.theta_p - self.theta,3)))
            MARKER_MANAGER.update_marker("length2", (4,-1.2), text = "length2=" + str(round((self.theta_p - self.theta)*self.r,3)))
        # ---- update path arc -----#
        theta_start = atan2((self.y - self.y_c) ,(self.x - self.x_c))
        theta_end = theta_start + self.w*DT

        # Arc can only draw in counter-clock wise direction
        if   sign(self.v) > 0 and self.dir_center == "RHS": 
            theta_end, theta_start = theta_start, theta_end # Swap
        elif sign(self.v) < 0 and self.dir_center == "RHS": 
            pass
        if   sign(self.v) > 0 and self.dir_center == "LHS":
            pass
        elif sign(self.v) < 0 and self.dir_center == "LHS":
            theta_end, theta_start = theta_start, theta_end # Swap

        MARKER_MANAGER.update_marker(self.marker_arc_name,
                                     (self.x_c, self.y_c),
                                     radius = self.r,
                                     angle_range = (theta_start, theta_end))

    def run_once(self):
        # Set x_p y_p
        self.x_p = CAR_MID_XYT[0]
        self.y_p = CAR_MID_XYT[1]
        
        # Assert check
        if  self.x == None or\
            self.y == None or\
            self.theta == None or\
            self.x_p == None or\
            self.y_p == None: 
            return False
        
        # Calculate inverse kinematics
        self.cal_IK()
        return True

    def car1_feedback_cb(self, data):
        '''
        '''
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        if data.pose.position.x != 0 or data.pose.position.y != 0: 
            data.pose.position.x = 0
            data.pose.position.y = 0
            self.x = 0
            self.y = 0
            self.theta = normalize_angle (euler[2])
            SERVER.setPose( data.marker_name, data.pose )
        else: 
            self.x     = data.pose.position.x
            self.y     = data.pose.position.y
            self.theta = normalize_angle (euler[2])
        
        # self.update_markers()
        SERVER.applyChanges()
    
    def car2_feedback_cb(self, data):
        '''
        '''
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)

        yaw = atan2(data.pose.position.y, data.pose.position.x)
        if data.pose.position.x != L * cos(yaw) or data.pose.position.y != L * sin(yaw):
            data.pose.position.x = L * cos(yaw)
            data.pose.position.y = L * sin(yaw)
            self.x = L * cos(yaw)
            self.y = L * sin(yaw)
            self.theta = normalize_angle (euler[2])
            SERVER.setPose( data.marker_name, data.pose )
        else: 
            self.x     = data.pose.position.x
            self.y     = data.pose.position.y
            self.theta = normalize_angle (euler[2])
        
        # self.update_markers()
        SERVER.applyChanges()



if __name__ == '__main__':
    #----- Init node ------# 
    rospy.init_node('kinematics_double_arc', anonymous=True)
    #----- Publisher -------#
    pub_car_1_result     = rospy.Publisher('car_1_result'  , PoseStamped ,queue_size = 10,  latch=True)
    pub_car_2_result     = rospy.Publisher('car_2_result'  , PoseStamped ,queue_size = 10,  latch=True)
    #------- Interactive Markers ---------# 
    SERVER = InteractiveMarkerServer("basic_controls")
    MARKER_MANAGER = Marker_Manager("markers")
   
    #--- Init cars -----# 
    #              x,y,theta,v,w
    car_1   = Car("start", (0,0,INIT_CAR1_THETA,0,0), "arc_center_start", "r_start")
    car_2   = Car("end", (0,0,INIT_CAR2_THETA,0,0), "arc_center_end", "r_end")
    (car_1.x, car_1.y) = (0, 0)
    (car_2.x, car_2.y) = (2.8284271247461903, 2.82842712474619)
    CAR_MID_XYT = (None, None, None)
    #------- interactive markers -----------# 
    q = tf.transformations.quaternion_from_euler(0, 0, car_1.theta)
    pose = Pose(Point(car_1.x,car_1.y,0), Quaternion(q[0],q[1],q[2],q[3])) # 0 degree
    make6DofMarker("start_pose", 0, "base_link", 1.0, (181, 101, 167), car_1.car1_feedback_cb,
                   False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True)
    q = tf.transformations.quaternion_from_euler(0, 0, car_2.theta)
    pose = Pose(Point(car_2.x,car_2.y,0), Quaternion(q[0],q[1],q[2],q[3]))# 45 degree
    make6DofMarker("end_pose", 0, "base_link", 1.0, (255, 111, 97), car_2.car2_feedback_cb,
                   False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True)
    pose = Pose(Point(0.5,0.5,0), Quaternion(0,0,0,1))
    make6DofMarker("car_m", 2, "base_link", 0.2, (255, 255, 0), car_m_feedback_cb,
                   False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True)

    # --- init text markers ---# 
    MARKER_MANAGER.register_marker("v1", 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("w1"  , 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("r1", 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("theta_p1"  , 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("length1"  , 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("v2", 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("w2"  , 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("r2", 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("theta_p2"  , 9 , "base_link", (255,255,255), 0.2)
    MARKER_MANAGER.register_marker("length2"  , 9 , "base_link", (255,255,255), 0.2)
    #---- init lines ------# Yellow line for arcs
    MARKER_MANAGER.register_marker("r_start", 4 , "base_link", (255,255,0), 0.02)
    MARKER_MANAGER.register_marker("r_end"  , 4 , "base_link", (255,255,0), 0.02)
    # Center of rotation
    # MARKER_MANAGER.register_marker("arc_center_start", 2 ,"base_link", (255,255,255), 0.2)
    # MARKER_MANAGER.register_marker("arc_center_end", 2 ,"base_link", (255,255,255), 0.2)

    #---- Init Markers ----# 
    init = InteractiveMarkerFeedback()
    init.marker_name = ""
    init.header.frame_id = "base_link"
    car_1.car1_feedback_cb(init)
    car_2.car2_feedback_cb(init)

    r = rospy.Rate(30) #call at 30HZ
    while (not rospy.is_shutdown()):
        rc1 = car_1.run_once()
        rc2 = car_2.run_once()
        if rc1 or rc2:
            cal_double_arc((car_1.x, car_1.y, car_1.theta), (car_2.x, car_2.y, car_2.theta))
            #---- update car_1 ----# 
            pub_car_1_result.publish(car_1.kinematic_result)
            #---- update car_2 -----# 
            pub_car_2_result.publish(car_2.kinematic_result)
            #---- update Textes -----# 
            # pub_marker_text.publish(marker_text)
            car_1.update_markers()
            car_2.update_markers()
            MARKER_MANAGER.publish()
            SERVER.applyChanges()
        r.sleep()

