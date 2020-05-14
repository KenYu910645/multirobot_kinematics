#!/usr/bin/env python
import rospy 
import sys
import time
import operator
import math
from math import sin, cos 
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

DT = 0.5 # sec
NUM_LINE_SEG = 30 # For arc
marker_sphere = MarkerArray()
marker_line   = MarkerArray()
marker_text   = MarkerArray()
is_text_need_publish = True 
L = 4
L_error = 0 

class Car():
    def __init__(self, id, init_kine): # unique id for every robot 
        self.id = id 
        (self.x,self.y,self.theta,self.v,self.w) = init_kine
        # ---- # 
        self.x_c = 0
        self.y_c = 0
        # ---- # 
        self.x_p = 0
        self.y_p = 0
        self.theta_p = 0
        self.kinematic_result = PoseStamped()
        #-- Publish ---#
        self.is_need_pub = True
        
    def cal_kinematics(self, dt):
        '''
        use x,y,theat  and v,w to calculate x_p, y_p, theta_p
        '''
        r = self.v/self.w
        # --- rotation center ----# 
        self.x_c = self.x - r*sin(self.theta)
        self.y_c = self.y + r*cos(self.theta)
        # --- result -----# 
        self.x_p = self.x_c + r*sin(self.theta + self.w*dt)
        self.y_p = self.y_c - r*cos(self.theta + self.w*dt)
        self.theta_p = self.theta + self.w*dt

        # ----- update kinematics result ------# 
        self.kinematic_result.header.frame_id = "base_link"
        self.kinematic_result.header.stamp = rospy.get_rostime()
        self.kinematic_result.pose.position.x = self.x_p
        self.kinematic_result.pose.position.y = self.y_p
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta_p)
        self.kinematic_result.pose.orientation.x = q[0]
        self.kinematic_result.pose.orientation.y = q[1]
        self.kinematic_result.pose.orientation.z = q[2]
        self.kinematic_result.pose.orientation.w = q[3]
        # ---- update rotation center -----# 
        set_sphere((self.x_c,self.y_c), (0,255,255) , 0.2, self.id)
        # ---- update arc -----#
        circle_points = [] 
        for i in range (NUM_LINE_SEG + 1):
            t = (dt/NUM_LINE_SEG)*i
            point = (self.x_c + r*sin(self.theta + self.w*t), self.y_c - r*cos(self.theta + self.w*t))
            circle_points.append(point)
        set_line(circle_points, (255,255,0), 0.02,self.id)
        # ----# 
        self.is_need_pub = True

def initial_pose_goal_CB(pose):
    pass 

def cal_L_error():
    global L_error, is_text_need_publish
    dx = car_1.x_p - car_2.x_p
    dy = car_1.y_p - car_2.y_p
    L_error = L - math.sqrt(dx**2 + dy**2)
    is_text_need_publish = True 

def marker_feedback_CB(data):
    global is_text_need_publish
    if data.marker_name[:3] == "car":
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        if data.marker_name == "car_1":
            car_1.x     = data.pose.position.x
            car_1.y     = data.pose.position.y
            car_1.theta = euler[2]
        elif data.marker_name == "car_2":
            car_2.x     = data.pose.position.x
            car_2.y     = data.pose.position.y
            car_2.theta = euler[2]

    elif data.marker_name == "v1":
        car_1.v = data.pose.position.y * 5
        set_text((-3,data.pose.position.y+1) , str(round(car_1.v ,2)) , (255,255,255) , 0.3, 1)
        is_text_need_publish = True 
    elif data.marker_name == "w1":
        car_1.w = data.pose.position.y * 5
        set_text((-3.5,data.pose.position.y+1) , str(round(car_1.w ,2)) , (255,255,255) , 0.3, 2)
        is_text_need_publish = True 
    elif data.marker_name == "v2":
        car_2.v = data.pose.position.y * 5
        set_text((-4,data.pose.position.y+1) , str(round(car_2.v ,2)) , (255,255,255) , 0.3, 3)
        is_text_need_publish = True 
    elif data.marker_name == "w2":
        car_2.w = data.pose.position.y * 5
        set_text((-4.5,data.pose.position.y+1) , str(round(car_2.w ,2)) , (255,255,255) , 0.3, 4)
        is_text_need_publish = True

    car_1.cal_kinematics(DT)
    car_2.cal_kinematics(DT)
    cal_L_error()
    set_text((0,-1) , "L_error = " + str(round(L_error,3)) , (255,255,255) , 0.3, 5)

def set_sphere(point , RGB = None  , size = 0.05, id = 0):
    '''
    Set Point at MarkArray 
    Input : 
        point - (x,y) or idx 
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
    # TODO, assign id 
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

#           id ,x,y,theta,v,w
car_1 = Car(1, (0,0,0,5,5))
car_2 = Car(2, (4,0,0,5,5))

def main(args):
    global is_text_need_publish
    #----- Init node ------# 
    rospy.init_node('kinematics', anonymous=True)
    #----- Subscribers ------# 
    rospy.Subscriber('/basic_controls/feedback', InteractiveMarkerFeedback, marker_feedback_CB)
    #----- Publisher -------# 
    pub_car_1_result    = rospy.Publisher('car_1_result', PoseStamped ,queue_size = 10,  latch=False)
    pub_car_2_result    = rospy.Publisher('car_2_result', PoseStamped ,queue_size = 10,  latch=False)
    #-- publish marker --# 
    pub_marker_sphere     = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=True)
    pub_marker_line       = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=True)
    pub_marker_text       = rospy.Publisher('marker_text'  , MarkerArray,queue_size = 1,latch=True)
    
    r = rospy.Rate(30) #call at 30HZ
    # --- init publish markers ---# 
    car_1.cal_kinematics(DT)
    car_2.cal_kinematics(DT)
    # --- init text markers ---# 
    set_text((-3,2) , str(round(car_1.v ,2)) , (255,255,255) , 0.3, 1)
    set_text((-3.5,2) , str(round(car_1.w ,2)) , (255,255,255) , 0.3, 2)
    set_text((-4,2) , str(round(car_2.v ,2)) , (255,255,255) , 0.3, 3)
    set_text((-4.5,2) , str(round(car_2.w ,2)) , (255,255,255) , 0.3, 4)
    set_text((0,-1) , "L_error = " + str(round(L_error,3)) , (255,255,255) , 0.3, 5)

    while (not rospy.is_shutdown()):
        if car_1.is_need_pub:
            pub_car_1_result.publish(car_1.kinematic_result)
            pub_marker_sphere.publish(marker_sphere)
            pub_marker_line.publish(marker_line)
            car_1.is_need_pub = False
        if car_2.is_need_pub:
            pub_car_2_result.publish(car_2.kinematic_result)
            pub_marker_sphere.publish(marker_sphere)
            pub_marker_line.publish(marker_line)
            car_2.is_need_pub = False
        if is_text_need_publish:
            pub_marker_text.publish(marker_text)
            is_text_need_publish = False
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
