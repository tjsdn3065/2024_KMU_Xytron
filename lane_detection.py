#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

image = np.empty(shape=[0]) 
bridge = CvBridge() 
motor = None 
img_ready = False 
Fix_Speed = 15

CAM_FPS = 30    
WIDTH, HEIGHT = 640, 480   
ROI_START_ROW = 300   
ROI_END_ROW = 380   
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW     
L_ROW = 40  
OFFSET = 20
OFFSET2 = 50
flag = False

pts = np.array([
    [OFFSET2, 0], # 좌상단
    [WIDTH - OFFSET2, 0], # 우상단
    [WIDTH, ROI_HEIGHT], # 우하단
    [OFFSET, ROI_HEIGHT] # 좌하단
], dtype=np.int32)

class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
            
    def get_sample_count(self):
        return len(self.data)
        
    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])

def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

def drive(angle, speed):
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
def lane_drive():
    global image, img_ready
    global motor
    prev_x_left = 0
    prev_x_right = WIDTH

    while img_ready == False:
        continue

    img = image.copy() 
    display_img = img  
    img_ready = False  

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)
    
    roi_img = img[ROI_START_ROW:ROI_END_ROW, OFFSET:WIDTH-OFFSET]
    
    roi_edge_img = edge_img[ROI_START_ROW:ROI_END_ROW, OFFSET:WIDTH-OFFSET]
    #mask = np.zeros_like(roi_edge_img)
    #cv2.fillPoly(mask, [pts], (255, 255, 255))
    #roi_edge_img = cv2.bitwise_and(roi_edge_img, mask)

    #cv2.imshow("roi edge img", roi_edge_img)

    #all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,30,50,20)
    all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,5,50,10) # thresh, minLineLength, maxLineGap
    
    if all_lines is None:
        return

    line_draw_img = roi_img.copy()
    
    slopes = []
    filtered_lines = []
    all_slope = []

    for line in all_lines:
        x1, y1, x2, y2 = line[0]

        if (x2 == x1):
            slope = 1000.0
        else:
            slope = float(y2-y1) / float(x2-x1)

        if 0.05 < abs(slope):
            slopes.append(slope)
            filtered_lines.append(line[0])

    if len(filtered_lines) == 0:
        return

    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = filtered_lines[j]
        slope = slopes[j]

        x1,y1, x2,y2 = Line

        Margin = 45
        
        if (slope < 0) and (x2 < WIDTH/2-Margin):
            left_lines.append(Line.tolist())

        elif (slope > 0) and (x1 > WIDTH/2+Margin):
            right_lines.append(Line.tolist())

    # print("Number of left lines : %d" % len(left_lines))
    # print("Number of right lines : %d" % len(right_lines))

    line_draw_img = roi_img.copy()
    
    # for line in left_lines:
    #     x1,y1, x2,y2 = line
    #     cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,0,255), 2)

    # for line in right_lines:
    #     x1,y1, x2,y2 = line
    #     cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,255,255), 2)

    m_left, b_left = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    size = len(left_lines)
    if size != 0:
        for line in left_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0                
            
        x_avg = x_sum / (size*2)
        y_avg = y_sum / (size*2)
        m_left = m_sum / size
        b_left = y_avg - m_left * x_avg

        if m_left != 0.0:
            x1 = int((0.0 - b_left) / m_left)        
            x2 = int((ROI_HEIGHT - b_left) / m_left)

            cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

    m_right, b_right = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    size = len(right_lines)
    if size != 0:
        for line in right_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0     
       
        x_avg = x_sum / (size*2)
        y_avg = y_sum / (size*2)
        m_right = m_sum / size
        b_right = y_avg - m_right * x_avg

        if m_right != 0.0:
            x1 = int((0.0 - b_right) / m_right)  
            x2 = int((ROI_HEIGHT - b_right) / m_right)

            cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

    if m_left == 0.0:
        x_left = prev_x_left	
    else:
        x_left = int((L_ROW - b_left) / m_left)
                        
    if m_right == 0.0:
        x_right = prev_x_right
    else:
        x_right = int((L_ROW - b_right) / m_right)
       
    if m_left == 0.0 and m_right != 0.0:
        x_left = x_right - 330

    if m_left != 0.0 and m_right == 0.0:
        x_right = x_left + 330
			
    prev_x_left = x_left
    prev_x_right = x_right
    
    x_midpoint = (x_left + x_right) // 2 
    view_center = WIDTH//2
  
    #cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
    #cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
    #cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
    #cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
    #cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

    display_img[ROI_START_ROW:ROI_END_ROW, OFFSET:WIDTH-OFFSET] = line_draw_img
    #cv2.imshow("Lanes positions", display_img)
    #cv2.waitKey(1)

    x = x_midpoint - view_center
    y = HEIGHT//2
    max_angle = 150
    angle = int(1 * math.atan(x / y) / (math.pi / 2) * max_angle)
    #angle = (x_midpoint-view_center) // 2
    speed = Fix_Speed
            
    drive(angle, speed)

def start():
    global image, img_ready
    global motor
    LANE = 1
    drive_mode = LANE

    rospy.init_node('lane_control')
    motor = rospy.Publisher('lane_control', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    #print ("----- Xycar self driving -----")

    #while not image.size == (WIDTH * HEIGHT * 3):
        #continue

    #print("Camera Ready --------------")

  
    while not rospy.is_shutdown():

       lane_drive()

if __name__ == '__main__':
    start()