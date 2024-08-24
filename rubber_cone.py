#!/usr/bin/env python3

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray
import numpy as np

class Xycar_Ctrl:
    def __init__(self):
        rospy.Subscriber("/lidar_cluster", PointCloud2, self.Lidar_Callback)
        self.motor_pub = rospy.Publisher('/cone_control', xycar_motor, queue_size=10)

        self.motor_msg = xycar_motor()
        self.max_angle = 200
        self.max_speed = 10

        self.waypoint_x = -1
        self.waypoint_y = -1

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.rubber_cone()
            rate.sleep()

    def Lidar_Callback(self, lidar_msg):
        point_generator = pc2.read_points(lidar_msg, field_names=("x", "y"), skip_nans=True)

        self.point_list = [] 
        for point in point_generator:
            self.point_list.append(point)

    def add_line_points(self, points, current_point, line, side):
        next_points = points[(points[:, 0] > current_point[0]) & (np.linalg.norm(points - current_point, axis=1) <= 0.4)]
        while next_points.size > 0:
            next_point = next_points[np.argmin(np.linalg.norm(next_points, axis=1))]
            if np.linalg.norm(next_point - current_point) <= 0.4:
                line.append(next_point)
                current_point = next_point
                next_points = points[(points[:, 0] > current_point[0]) & (np.linalg.norm(points - current_point, axis=1) <= 0.4)]
            else:
                break

    def get_index_based_center_point(self, left_line, right_line):
        left_size = len(left_line)
        right_size = len(right_line)

        if left_size == 0 and right_size == 0:
            return None, None
        elif left_size == 0:
            return None, right_line[-1]  # 오른쪽 라인의 마지막 포인트를 반환
        elif right_size == 0:
            return left_line[-1], None  # 왼쪽 라인의 마지막 포인트를 반환

        # 라인 간의 포인트 차이 계산
        difference = left_size - right_size
        n=abs(difference)
        if(n > 4):
            n = 4

        # 차이에 따른 중앙 포인트 설정
        if difference == 0:
            # 라인 길이가 같은 경우 각 라인의 첫 번째 포인트를 사용
            return left_line[0], right_line[0]
        elif difference > 0:
            # 왼쪽 라인이 더 긴 경우
            return left_line[n], right_line[0]
        else:
            # 오른쪽 라인이 더 긴 경우
            return left_line[0], right_line[n]

        # 위의 조건에 해당하지 않는 경우 (보안)
        return None, None


    def rubber_cone(self):
        if self.point_list:  # self.point_list가 비어있지 않은지 추가로 확인
            points = np.array(self.point_list)
            if points.size == 0:
                rospy.loginfo("No points received yet.")
                return
            
            # 왼쪽과 오른쪽 포인트 초기화 및 라인 구성
            left_line, right_line = [], []

            # 왼쪽 포인트 초기화
            left_points = points[(points[:, 1] > 0) & (points[:, 0] > 0)]
            if left_points.size > 0:
                current_point = left_points[np.argmin(np.linalg.norm(left_points, axis=1))]
                if np.linalg.norm(current_point) < 0.7:
                    left_line.append(current_point)
                    self.add_line_points(points, current_point, left_line, 'left')

            # 오른쪽 포인트 초기화
            right_points = points[(points[:, 1] < 0) & (points[:, 0] > 0)]
            if right_points.size > 0:
                current_point = right_points[np.argmin(np.linalg.norm(right_points, axis=1))]
                if np.linalg.norm(current_point) < 0.7:
                    right_line.append(current_point)
                    self.add_line_points(points, current_point, right_line, 'right')

            # rospy.loginfo(f"Left line points: {left_line}")
            # rospy.loginfo(f"Right line points: {right_line}")

            # 각 라인의 포인트 개수 기반 중앙 포인트 추출
            left_center_point, right_center_point = self.get_index_based_center_point(left_line, right_line)

            if left_center_point is None and right_center_point is None:
                # rospy.logwarn("No valid points found, stopping")
                self.motor_msg.angle = 0
                self.motor_msg.speed = 0
            else:
                if left_center_point is not None and right_center_point is None:
                    # rospy.loginfo("Only left point detected, turning right")
                    self.motor_msg.angle = self.max_angle  # 오른쪽으로 큰 각도로 조향
                    self.motor_msg.speed = self.max_speed
                elif left_center_point is None and right_center_point is not None:
                    # rospy.loginfo("Only right point detected, turning left")
                    self.motor_msg.angle = -self.max_angle  # 왼쪽으로 큰 각도로 조향
                    self.motor_msg.speed = self.max_speed
                else:
                    self.waypoint_x = (left_center_point[0] + right_center_point[0]) / 2.0
                    self.waypoint_y = (left_center_point[1] + right_center_point[1]) / 2.0

                    if self.waypoint_x != 0:
                        self.motor_msg.angle = int(math.atan(-1 * self.waypoint_y / self.waypoint_x) / (math.pi / 2) * self.max_angle)
                    else:
                        self.motor_msg.angle = 0 if self.waypoint_y >= 0 else -self.max_angle  # 방향에 따라 최대 각도 조정
                    # rospy.loginfo("angle: %d", self.motor_msg.angle)
                    self.motor_msg.speed = self.max_speed
                
            self.motor_pub.publish(self.motor_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('rubber_cone')
        xycar = Xycar_Ctrl()

    except rospy.ROSInternalException:
        pass 
