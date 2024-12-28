#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

pi = 3.14158265

def boundRect(contours) :
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])
    return boundRect


def dectectBlack(bgrimage) : 
    rgbimage = cv2.cvtColor(bgrimage, cv2.COLOR_BGR2RGB)
    hsvimage = cv2.cvtColor(bgrimage, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsvimage, (0, 0, 0), (255, 255, 0))
    mask_L = mask1[0:480, 240:320]
    mask_R = mask1[0:480, 320:400]
    contours1, hierarchy = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_L, hierarchy = cv2.findContours(mask_L, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_R, hierarchy = cv2.findContours(mask_R, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    xstart = 640
    ystart = 640
    xend = 0
    yend = 0
    detected = False
    in_front = False
    for (x, y, width, height) in boundRect(contours1) :
        if x < xstart:
            xstart = x
            detected = True
        if y <ystart :
            ystart = y
            detected = True
        if x + height > xend :
            xend = x + height
            detected = True
        if y + width > yend :
            yend = y + width
            detected = True
    cv2.drawContours(rgbimage, contours1, -1, (0,255,0), 5)
    if detected == True :
        cv2.rectangle(rgbimage, (xstart, ystart), (xend, yend), (0,0,255), 5)
        rectangle_area = (xend - xstart)*(yend-ystart)
        if rectangle_area >= 480*480 :
            in_front = True
    
    cv2.imshow("Image window", rgbimage)
    cv2.waitKey(3)
    
    if len(contours_L) > 0 and len(contours_R) > 0:
        if in_front :
            rospy.loginfo("object in front")
            return 2
        else :
            rospy.loginfo("detected")
            return 1
    else :
        rospy.loginfo("undetected")
        return 0

class image_reader:
    def __init__(self):
        self.move_pub = rospy.Publisher("/basic_bot/cmd_vel",Twist, queue_size=10)
        self.joint_pub1 = rospy.Publisher('/basic_bot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint_pub2 = rospy.Publisher('/basic_bot/joint2_position_controller/command', Float64, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/basic_bot/camera1/image_raw",Image,self.callback,queue_size=1)
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            pass
            
        move = Twist()
        move.linear.x = 0
        move.linear.y = 0
        move.linear.z = 0
        move.angular.x = 0
        move.angular.y = 0
        move.angular.z = -1

        if dectectBlack(cv_image) == 2 :
            move.angular.z = 0
            self.move_pub.publish(move)
            rospy.sleep(0.5)
            self.joint_pub1.publish(Float64(0))
            self.joint_pub2.publish(Float64(pi))
            rospy.sleep(0.25)
            self.joint_pub1.publish(Float64(pi))
            self.joint_pub2.publish(Float64(pi))
            rospy.sleep(0.5)
            self.joint_pub1.publish(Float64(0))
            self.joint_pub2.publish(Float64(0))
        else :
            if dectectBlack(cv_image) == 1 :
                move.linear.x = 1 
                move.angular.z = 0
        
            try:
                self.move_pub.publish(move)
                self.joint_pub1.publish(Float64(0))
                self.joint_pub2.publish(Float64(0))
            except rospy.ROSInterruptException:
                pass
            
    

if __name__ == '__main__':
    try:
        ir = image_reader()
        rospy.init_node('move_when_black', anonymous=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

