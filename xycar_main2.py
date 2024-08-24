#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2, rospy, time, math, os
import numpy as np
from xycar_msgs.msg import xycar_motor
from yolov5_ROS.msg import Yolo_Objects
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int16

class xycar:
    def __init__(self):
        rospy.init_node('xycar_main', anonymous=True)

        rospy.Subscriber('/yolov5_pub', Yolo_Objects, self.yolo_callback)
        self.is_yolo= False
        self.yolo_msg = None

        rospy.Subscriber('/ar_control', xycar_motor, self.ar_control_callback)
        self.is_ar_control=False
        self.ar_control_msg=None

        rospy.Subscriber('/lane_control', xycar_motor, self.lane_control_callback)
        self.is_lane_control=False
        self.lane_control_msg=None

        rospy.Subscriber('/cone_control', xycar_motor, self.cone_control_callback)
        self.is_cone_control=False
        self.cone_control_msg=None

        rospy.Subscriber('/obstacle_position_check', Int16, self.obstacle_position_check_callback)
        self.is_obstacle_position_check=False
        self.obstacle_position_check_msg=None

        rospy.Subscriber("/lidar_cluster", PointCloud2, self.lidar_Callback)
        self.is_lidar=False

        self.cmd_vel_pub = rospy.Publisher("/xycar_motor", xycar_motor, queue_size=1)
        self.motor = xycar_motor()

        self.ar_flag=False
        self.traffic_flag=False
        self.lane_flag=False
        ##### cone ######
        self.cone_start=False
        self.cone_in_progress=False
        self.cone_end=False
        ##### obstacle ######
        self.obstacle_start=False
        self.obstacle_end=True

        self.red_traffic_not_detected_count = 0
        self.ar_marker_not_detected_count = 0  # 연속적으로 AR 마커를 감지하지 못한 횟수를 추적
        self.cone_not_detected_count = 0
        self.first_cone_detected_count = 0
        self.obstacle_detected_count = 0

        # 시간 측정을 위한 변수 추가
        self.start_time = None  # 시간 측정 시작 시간
        self.elapsed_time = 0  # 경과 시간
        self.time_flag=False

        self.red_detected = False
        self.cone_detected = False

        self.right_avoidance=False
        self.left_avoidance=False
        self.center_avoidance=False

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_yolo and self.is_lane_control and self.is_ar_control and self.is_cone_control and self.is_lidar and self.is_obstacle_position_check: # 각 토픽이 들어오고 있는지 확인

                if self.ar_flag == False: # ar 미션 구간이 끝나지 않았다면

                    if self.traffic_flag == False: # 신호등 미션 구간이 끝나지 않았다면

                        if self.check_green_traffic_light_detected(): # 녹색 신호가 감지되었다면 -> 신호등 미션 구간 끝
                            self.traffic_flag = True

                        elif self.check_red_traffic_light_detected() == False: # 적색 신호가 감지되지 않는다면(대회 규정을 보면 자이카를 실행시켜놓고 카메라를 가린 상태로 대기하기 때문에 정지 필요) -> 정지
                            self.motor.speed=0
                            self.motor.angle=0

                        elif self.check_nearby_red_traffic_light_detected(): # 적색 신호가 가까워 졌다면 -> 정지
                            self.motor.speed=0
                            self.motor.angle=0

                        elif self.check_red_traffic_light_detected(): # 적색 신호가 감지된다면 ar마커 따라서 신호등 쪽으로 이동(처음 출발을 위해 전진)
                            self.motor=self.ar_control_msg

                    else: # 신호등 미션 구간이 끝났다면 -> ar 미션 주행, 시간 측정으로 한다면 여기에 추가

                        if self.time_flag == False:
                            self.start_timer()
                            self.time_flag = True

                        self.elapsed_time = time.time() - self.start_time

                        if self.elapsed_time > 20:
                            print("ar stop")
                            self.motor.speed=0
                            self.motor.angle=0
                            self.cmd_vel_pub.publish(self.motor) # 2초간 정지 후 출발
                            time.sleep(2)
                            self.ar_flag = True
                            self.time_flag = False
                        else:
                            self.motor=self.ar_control_msg

                    self.cmd_vel_pub.publish(self.motor)

                else: # ar 미션 구간이 끝났다면 -> 차선 주행

                    print("start")
                    if self.obstacle_end and self.cone_start == False and self.first_check_rubber_cone_detected and self.rubber_cone_detected_from_point(): # 장애물 구간이 끝났고 라바콘을 시작하지 않았고 라바콘 큰 게 검출되었고 클러스터링이 전방에 많다면 -> 라바콘 미션 시작
                        self.cone_start = True
                        self.cone_in_progress = True
                    elif self.cone_in_progress and self.check_rubber_cone_detected(): # 라바콘 진행 중이고 라바콘이 검출되었다면 -> 라바콘 진행 중
                        self.motor=self.cone_control_msg
                    elif self.cone_in_progress and self.check_rubber_cone_detected() == False: # 라바콘 진행 중이고 라바콘이 검출되지 않는다면 -> 라바콘 미션 끝
                        self.cone_start = False
                        self.cone_in_progress = False
                        self.obstacle_end = False
                        self.cone_end = True
                    elif self.cone_end and self.obstacle_start == False and self.check_obstacle_detected() and self.obstacle_detected_from_point(): # 라바콘 구간이 끝났고 장애물을 시작하지 않았고 장애물이 검출되었고 클러스터링이 검출된다면 -> 장애물 미션 시작
                        self.obstacle_start = True
                    elif self.obstacle_start:
                        if self.left_avoidance:      # 장애물 회피 시작
                            print("left")
                            # 장애물 회피 제어
                            for _ in range(1):
                                self.motor.angle=0
                                self.motor.speed=5
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(12):
                                self.motor.angle=-35
                                self.motor.speed=15
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(12):
                                self.motor.angle=25
                                self.motor.speed=15
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            self.left_avoidance=False
                        elif self.right_avoidance:
                            print("right")
                            # 장애물 회피 제어
                            for _ in range(1):
                                self.motor.angle=0
                                self.motor.speed=5
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(11):
                                self.motor.angle=35
                                self.motor.speed=15
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(12):
                                self.motor.angle=-25
                                self.motor.speed=15
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            self.right_avoidance=False
                        elif self.center_avoidance:
                            print("center")
                            # 장애물 회피 제어
                            for _ in range(1):
                                self.motor.angle=0
                                self.motor.speed=5
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(15):
                                self.motor.angle=-50
                                self.motor.speed=10
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            for _ in range(16):
                                self.motor.angle=35
                                self.motor.speed=10
                                self.cmd_vel_pub.publish(self.motor)
                                time.sleep(0.1)
                            self.center_avoidance=False
                                
                        self.obstacle_start = False
                        self.obstacle_end = True
                        self.cone_end = False
                        
                    elif self.obstacle_start == False and self.cone_start == False:             # 장애물 라바콘 둘 다 시작하지 않았으면 차선
                        print("lane_control")
                        self.motor = self.lane_control_msg
                        #self.motor.angle=0
                        #self.motor.speed=0
                    self.cmd_vel_pub.publish(self.motor)
            else:
                print(f"/bounding_boxes is {self.is_yolo}")
                print(f"/ar_control is {self.is_ar_control}")
                print(f"/lane_control is {self.is_lane_control}")
                print(f"/cone_control is {self.is_cone_control}")
                print(f"/lidar_cluster is {self.is_lidar}")
                print(f"/obstacle_position_check is {self.is_obstacle_position_check}")

            rate.sleep()
        
    def yolo_callback(self,msg):
        self.is_yolo=True
        self.yolo_msg=msg

    def ar_control_callback(self,msg):
        self.is_ar_control=True
        self.ar_control_msg=msg

    def lane_control_callback(self,msg):
        self.is_lane_control=True
        self.lane_control_msg=msg

    def cone_control_callback(self,msg):
        self.is_cone_control=True
        self.cone_control_msg=msg

    def obstacle_position_check_callback(self,msg):
        self.is_obstacle_position_check=True
        self.obstacle_position_check_msg=msg.data

    def lidar_Callback(self, lidar_msg):
        self.is_lidar=True
        self.point_list = []
        point_generator = pc2.read_points(lidar_msg, field_names=("x", "y"), skip_nans=True)

        for point in point_generator:
            self.point_list.append(point)

    def rubber_cone_detected_from_point(self):  # 클러스터링으로 라바콘 미션 판단 함수
        count = 0  # 조건을 만족하는 점의 수를 세는 변수

        # 클러스터링된 포인트들 중 특정 조건을 검사
        for point in self.point_list:
            x, y = point
            if 0 < x < 1.0 and -0.6 < y < 0.6:
                count += 1
                if count >= 6:
                    return True  # 조건을 만족하는 포인트가 6개 이상일 경우 True 반환
                
        return False  # 6개 미만일 경우 False 반환
    
    def obstacle_detected_from_point(self):  # 클러스터링으로 장애물 미션 판단 함수
        closest_point = None
        min_distance_sq = float('inf')  # 최소 거리의 제곱을 무한대로 초기화

        # 조건에 맞는 가장 가까운 점을 찾는 과정
        for point in self.point_list:
            x, y = point
            if 0 < x < 1.5 and -0.2 < y < 0.2:
                distance_sq = x**2 + y**2  # 제곱 거리 계산
                if distance_sq < min_distance_sq:
                    min_distance_sq = distance_sq
                    closest_point = point  # 가장 가까운 점 업데이트

        # 가장 가까운 점을 기준으로 장애물 감지 로직 수행
        if closest_point:
            x, y = closest_point
            if 0 < x < 1.0:
                if self.obstacle_position_check_msg == 0:
                    self.right_avoidance=True
                    return True  # Obstacle detected
                elif self.obstacle_position_check_msg == 2:
                    self.left_avoidance=True
                    return True  # Obstacle detected
                elif self.obstacle_position_check_msg == 1:
                    self.center_avoidance=True
                    return True  # Obstacle detected

        return False  # 조건을 만족하는 가장 가까운 점이 없거나 장애물 미감지

    def start_timer(self):
        """시간 측정을 시작합니다."""
        self.elapsed_time = 0
        self.start_time = time.time()

    def stop_timer(self):
        """시간 측정을 종료하고 경과 시간을 업데이트합니다."""
        if self.start_time is not None:
            self.elapsed_time = time.time() - self.start_time
            self.start_time = None
        
    def check_red_traffic_light_detected(self):
        """감지된 메시지에서 빨간 신호등이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환
        
        red_detected = False

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 0:
                red_detected = True
                self.red_detected=True
                self.red_traffic_not_detected_count = 0
                break  # 빨간 신호등 발견

        if red_detected == False and self.red_detected:
            # 감지되지 않았으면 카운터 증가
            self.red_traffic_not_detected_count += 1

        # 5번 이상 연속으로 감지되지 않은 경우
        if self.ar_marker_not_detected_count >= 5:
            self.red_detected=False
            return False

        return self.red_detected
    
    def check_nearby_red_traffic_light_detected(self):
        """감지된 메시지에서 가까운 빨간 신호등이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 0:
                width = bbox.x2 - bbox.x1
                height = bbox.y2 - bbox.y1
                area = width * height
                if area >= 9000:
                    return True  # 빨간 신호등 발견

        return False  # 빨간 신호등이 없으면 False 반환
    
    def check_green_traffic_light_detected(self):
        """감지된 메시지에서 녹색 신호등이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 1:
                return True  # 녹색 신호등 발견

        return False  # 녹색 신호등이 없으면 False 반환
    
    def check_ar_marker_detected(self):
        """감지된 메시지에서 ar마커가 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환

        marker_detected = False
        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 2:
                print("AR marker detected")
                marker_detected = True
                self.ar_marker_not_detected_count = 0  # 마커 감지 시 카운터 초기화
                break

        if not marker_detected:
            # AR 마커가 감지되지 않았으면 카운터 증가
            self.ar_marker_not_detected_count += 1

        # 10번 이상 연속으로 감지되지 않은 경우
        if self.ar_marker_not_detected_count >= 10:
            return False

        return marker_detected
    
    def first_check_rubber_cone_detected(self):
        """감지된 메시지에서 라바콘이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 3:
                width = bbox.x2 - bbox.x1
                height = bbox.y2 - bbox.y1
                area = width * height
                if area >= 8000:
                    self.cone_detected_count+=1
                    if self.cone_detected_count>=5:
                        return True  # 라바콘 발견
                else:
                    self.cone_detected_count=0
                    break

        return False  # 라바콘이 없으면 False 반환
    
    def check_rubber_cone_detected(self):
        """감지된 메시지에서 라바콘이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환
        
        cone_detected = False

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 3:
                cone_detected = True
                self.cone_detected=True
                self.cone_not_detected_count = 0
                break  # 라바콘 발견

        if cone_detected == False and self.cone_detected:
            # 라바콘이 감지되지 않았으면 카운터 증가
            self.cone_not_detected_count += 1

        # 5번 이상 연속으로 감지되지 않은 경우
        if self.cone_not_detected_count >= 5:
            self.cone_detected = False
            return False

        return self.cone_detected
    
    def check_obstacle_detected(self):
        """감지된 메시지에서 장애물이 있는지 확인하고 결과를 반환하는 함수"""
        if self.yolo_msg is None:
            return False  # 아직 감지된 메시지가 없으면 False 반환
        
        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 4:    
                width = bbox.x2 - bbox.x1
                height = bbox.y2 - bbox.y1
                area = width * height
                if area >= 4000:
                    self.obstacle_detected_count+=1
                    if self.obstacle_detected_count>=5:
                        return True
                else:
                    self.obstacle_detected_count=0
                    break

        return False  # 장애물이 없으면 False 반환
    
if __name__ == '__main__':
    try:
        xycar()
    except rospy.ROSInterruptException:
        pass
