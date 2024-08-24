#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from yolov5_ROS.msg import Yolo_Objects, Objects
from std_msgs.msg import Int16

# image shape
Width = 640
Height = 480

# warp shape
warp_img_w = 640
warp_img_h = 480

# warp parameter
x_h = 100
x_l = 150
y_h = 80
y_l = 80

# line
Margin = 150

# Center
Offset = 40

warp_src = np.array([
    [x_h, Height//2 + y_h], # 좌상단
    [-x_l, Height - y_l], # 좌하단
    [Width - x_h, Height//2 + y_h], # 우상단
    [Width + x_l, Height - y_l] # 우하단
], dtype=np.float32)

warp_dst = np.array([
    [0, 0],
    [0, warp_img_h],
    [warp_img_w, 0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

class CameraReceiver:
    def __init__(self):
        rospy.loginfo("Camera Receiver Object is Created")
        self.bridge = CvBridge()

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        rospy.Subscriber("/yolov5_pub", Yolo_Objects, self.yolo_callback)

        self.lane_check_pub = rospy.Publisher("/obstacle_position_check", Int16, queue_size=1)
        self.lane_check_msg = Int16()

        self.car_x = -1
        self.car_y = -1
    
    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        edge_img = edge(image)
        warp_img, M, Minv = warp(edge_img, warp_src, warp_dst, (warp_img_w, warp_img_h))
        #line_img, mid_x_L, mid_x_R, mid_x_C = process_lines(warp_img)
        result = process_lines(warp_img)

        if result is None:
            rospy.logwarn("No lines detected")
            return
    
        line_img, mid_x_L, mid_x_R, mid_x_C = result
        #rewarp_img = rewarp(image, line_img, Minv)

        if mid_x_L != -1 or mid_x_R != -1 or mid_x_C != -1:
            #point_L = np.array([[mid_x_L, Width//2]], dtype='float32')
            #point_L_t = cv2.perspectiveTransform(np.array([point_L]), Minv)[0]
            #cv2.circle(rewarp_img, (point_L_t[0][0],point_L_t[0][1]), 15, (0, 0, 255), -1)

            #point_R = np.array([[mid_x_R, Width//2]], dtype='float32')
            #point_R_t = cv2.perspectiveTransform(np.array([point_R]), Minv)[0]
            #cv2.circle(rewarp_img, (point_R_t[0][0],point_R_t[0][1]), 15, (0, 255, 255), -1)

            point_C = np.array([[mid_x_C, Width//2]], dtype='float32')
            point_C_t = cv2.perspectiveTransform(np.array([point_C]), Minv)[0]
            #cv2.circle(rewarp_img, (point_C_t[0][0],point_C_t[0][1]), 15, (255, 0, 0), -1)

        if self.car_x != -1 and self.car_y != -1:
            #cv2.circle(rewarp_img, (self.car_x, self.car_y), 15, (255, 255, 0), -1)
            #rospy.loginfo("Car Detected")

            if self.car_x < point_C_t[0][0] - Offset:
                rospy.loginfo("Left")
                self.lane_check_msg.data = 0

            elif point_C_t[0][0] + Offset < self.car_x:
                rospy.loginfo("Right")
                self.lane_check_msg.data = 2

            else:
                rospy.loginfo("Center")
                self.lane_check_msg.data = 1

            self.lane_check_pub.publish(self.lane_check_msg)

        #cv2.imshow("edge", edge_img)
        #cv2.imshow("warp", warp_img)
        #cv2.imshow("line", line_img)
        #cv2.imshow("rewarp_img", rewarp_img)
        #cv2.waitKey(1)

    def yolo_callback(self, data):
        yolo_objects = data.yolo_objects

        for obj in yolo_objects:
            if obj.Class == 4:
                self.car_x = (obj.x1 + obj.x2) // 2
                self.car_y = (obj.y1 + obj.y2) // 2
                
def warp(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def edge(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur), 100, 150)
        
    return edge_img

def find_best_line(lines):
    if not lines:
        return None

    best_line = None
    min_distance_to_center = float('inf')
    
    for line in lines:
        x1, y1, x2, y2 = line
        mid_x = (x1 + x2) / 2
        distance_to_center = abs(mid_x - Width/2)
        
        # Find the line closest to the center
        if distance_to_center < min_distance_to_center:
            min_distance_to_center = distance_to_center
            best_line = line
    
    return best_line

def extend_line(x1, y1, x2, y2, extension=1000):
    # Calculate the direction vector
    line_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    dx = (x2 - x1) / line_length
    dy = (y2 - y1) / line_length

    # Extend the line by the specified amount in both directions
    x1_ext = int(x1 - dx * extension)
    y1_ext = int(y1 - dy * extension)
    x2_ext = int(x2 + dx * extension)
    y2_ext = int(y2 + dy * extension)

    return x1_ext, y1_ext, x2_ext, y2_ext

def process_lines(gray_img):
    line_img = np.dstack((gray_img, gray_img, gray_img)) * 255
    #line_img = np.zeros((Height, Width, 3), dtype=np.uint8)

    all_lines = cv2.HoughLinesP(gray_img, 1, math.pi/180, 5, 50, 10)

    if all_lines is None:
        return None

    slopes = []
    filtered_lines = []

    for line in all_lines:
        x1, y1, x2, y2 = line[0]

        if x2 == x1:
            slope = 1000.0  # Handle vertical lines
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if 20 < abs(slope):  # Filter out nearly horizontal lines
            slopes.append(slope)
            filtered_lines.append(line[0])

    if len(filtered_lines) == 0:
        return None

    left_lines = []
    right_lines = []
    center_lines = []

    for j in range(len(slopes)):
        line = filtered_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = line

        if (x2 < Width / 2 - Margin):
            left_lines.append(line.tolist())

        elif (x1 > Width / 2 + Margin):
            right_lines.append(line.tolist())

        elif ((Width / 2 - Margin < x1 < Width / 2 + Margin) and 
              (Width / 2 - Margin < x2 < Width / 2 + Margin)):
            center_lines.append(line.tolist())

    # Find and draw the most suitable lines with extension
    best_left_line = find_best_line(left_lines)
    best_right_line = find_best_line(right_lines)
    best_center_line = find_best_line(center_lines)

    mid_x_L = -1
    mid_x_R = -1
    mid_x_C = -1

    if best_left_line is not None:
        x1, y1, x2, y2 = best_left_line
        #x1_ext, y1_ext, x2_ext, y2_ext = extend_line(x1, y1, x2, y2)
        mid_x_L = (x1 + x2) // 2
        #mid_y = Height // 2
        #cv2.line(line_img, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 0, 255), 10)
        #cv2.circle(line_img, (mid_x_L, mid_y), 15, (0, 0, 255), -1)

    if best_right_line is not None:
        x1, y1, x2, y2 = best_right_line
        #x1_ext, y1_ext, x2_ext, y2_ext = extend_line(x1, y1, x2, y2)
        mid_x_R = (x1 + x2) // 2
        #mid_y = Height // 2
        #cv2.line(line_img, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 255, 255), 10)
        #cv2.circle(line_img, (mid_x_R, mid_y), 15, (0, 255, 255), -1)

    if best_center_line is not None:
        x1, y1, x2, y2 = best_center_line
        #x1_ext, y1_ext, x2_ext, y2_ext = extend_line(x1, y1, x2, y2)
        mid_x_C = (x1 + x2) // 2
        #mid_y = Height // 2
        #cv2.line(line_img, (x1_ext, y1_ext), (x2_ext, y2_ext), (255, 0, 0), 10)
        #cv2.circle(line_img, (mid_x_C, mid_y), 15, (255, 0, 0), -1)

    return line_img, mid_x_L, mid_x_R, mid_x_C

def rewarp(image, warp_img, Minv):
    newwarp = cv2.warpPerspective(warp_img, Minv, (Width, Height))
    rewarp_img = cv2.addWeighted(image, 1, newwarp, 0.5, 0)

    return rewarp_img

def run():
    rospy.init_node("obstacle_position_check")
    cam = CameraReceiver()
    rospy.spin()

if __name__ == "__main__":
    try:
        run()
        
    except rospy.ROSInterruptException:
        pass
