#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy, math, time
from xycar_msgs.msg import xycar_motor
from yolov5_ROS.msg import Yolo_Objects, Objects
from ar_track_alvar_msgs.msg import AlvarMarkers


# from tf.transformations import euler_from_quaternion

class ARDriver:
    def __init__(self):
        rospy.init_node('ar_follow')
        self.motor = rospy.Publisher('/ar_control', xycar_motor, queue_size=1)
        self.motor_msg = xycar_motor()
        self.ar_msg = {"ID": [], "DX": [], "DZ": []}

        try:
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size=1)
            rospy.Subscriber('/yolov5_pub', Yolo_Objects, self.yolo_callback)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to subscribe to topics: {str(e)}")

        self.yolo_msg = None
        self.angle = 0
        self.speed = 7
        self.variable = 0.4  # ar마커가 특정 거리 이내에 들어왔을 때 스쳐지나가기 위한 변수
        self.dis = 2.0  # ar 마커 특정 거리 변수
        self.add_x_pos = -1.0  # ar 마커가 특정 거리 이내에 들어오기 전에 ar마커 위치로 가기 위한 변수
        self.add_angle = 150  # 조향각도 곱 변수

        self.cur_id = None
        self.prev_id = None

        self.green_light_detected = False

        self.pass_first_ar = False
        self.pass_second_ar = False
        self.pass_third_ar = False
        self.pass_fourth_ar = False

    def yolo_callback(self, msg):
        self.yolo_msg = msg

    def ar_callback(self, msg):
        self.ar_msg["ID"] = [marker.id for marker in msg.markers]
        self.ar_msg["DX"] = [marker.pose.pose.position.x for marker in msg.markers]
        self.ar_msg["DZ"] = [marker.pose.pose.position.z for marker in msg.markers]
        # self.ar_msg["O_X"] = [marker.pose.pose.orientation.x for marker in msg.markers]
        # self.ar_msg["O_Y"] = [marker.pose.pose.orientation.y for marker in msg.markers]
        # self.ar_msg["O_Z"] = [marker.pose.pose.orientation.z for marker in msg.markers]
        # self.ar_msg["O_W"] = [marker.pose.pose.orientation.w for marker in msg.markers]

    def check_AR(self):
        if not self.ar_msg["ID"]:
            return 99, 5.0, 5.0

        min_dz = float('inf')
        closest_id = 99
        closest_z = 5.0
        closest_x = 5.0

        try:
            for id, dx, dz in zip(self.ar_msg["ID"], self.ar_msg["DX"], self.ar_msg["DZ"]):
                # distance = math.sqrt(dx ** 2 + dz ** 2)
                if dz < min_dz:
                    min_dz = dz
                    closest_id = id
                    closest_z = dz
                    closest_x = dx
                    # quaternion=(ox, oy, oz, ow)
                    # _,pitch,_ = euler_from_quaternion(quaternion)
                    # pitch=pitch*180/math.pi
        except ValueError as e:
            rospy.logerr(f"Math error in check_AR: {str(e)}")
            return 99, 5.0, 5.0

        return closest_id, closest_z, closest_x

    def check_green_traffic_light_detected(self):
        if self.yolo_msg is None:
            return False

        for bbox in self.yolo_msg.yolo_objects:
            if bbox.Class == 1:
                return True  # 녹색 신호등 발견

        return False

    def drive(self, angle, speed):
        self.motor_msg.angle = int(angle)
        self.motor_msg.speed = int(speed)
        self.motor.publish(self.motor_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            try:
                self.cur_id, z_pos, x_pos = self.check_AR()  # 가장 가까운 ar마커 정보 취득
                # print(f"pitch: {pitch}")

                if self.green_light_detected == False and self.check_green_traffic_light_detected():  # 신호등이 초록색이면 -> 변수 값 변경
                    # print("its_me")
                    self.green_light_detected = True
                    self.variable = -0.50
                    self.dis = 1.0
                    self.add_x_pos = -0.1
                    self.add_angle = 200

                if self.cur_id == 99 and self.pass_third_ar and self.pass_fourth_ar == False:  # ar마커를 찾지 못했고 세번째 ar마커를 통과했고 네번째 ar 마커를 통과하지 않았을 때
                    self.angle = -20

                elif self.cur_id == 99 and self.pass_second_ar and self.pass_third_ar == False:  # ar마커를 찾지 못했고 두번째 ar마커를 통과했고 세번째 ar 마커를 통과하지 않았을 때
                    self.angle = 20
                elif self.cur_id == 99 and self.pass_first_ar and self.pass_second_ar == False and self.prev_id != None:  # ar마커를 찾지 못했고 첫번째 ar마커를 통과했고 두번째 ar 마커를 통과하지 않았을 때
                    self.angle = 15

                elif self.cur_id == 99:  # ar마커를 찾지 못했을 때
                    self.angle = 0

                else:  # ar 마커를 찾았다면

                    if self.pass_third_ar and self.pass_fourth_ar == False:  # 3~4 사이
                        if self.prev_id != None and self.prev_id != self.cur_id:  # 네번째 ar을 통과했다면
                            self.pass_fourth_ar = True
                            self.prev_id = None
                            self.dis = 1.0
                        elif z_pos < 1.3:
                            self.prev_id = self.cur_id

                    elif self.pass_second_ar and self.pass_third_ar == False:  # 2~3 사이
                        if self.prev_id != None and self.prev_id != self.cur_id:  # 세번째 ar을 통과했다면
                            self.pass_third_ar = True
                            self.prev_id = None
                            self.dis = 1.35
                        elif z_pos < 1.3:
                            self.prev_id = self.cur_id

                    elif self.pass_first_ar and self.pass_second_ar == False:  # 1~2 사이
                        if self.prev_id != None and self.prev_id != self.cur_id:  # 두번째 ar을 통과했다면
                            self.pass_second_ar = True
                            self.prev_id = None
                            self.dis = 0.95
                        elif z_pos < 1.3:
                            self.prev_id = self.cur_id

                    elif self.green_light_detected and self.pass_first_ar == False:  # 0~1 사이
                        if self.prev_id != None and self.prev_id != self.cur_id:  # 첫번째 ar을 통과했다면
                            self.pass_first_ar = True
                            self.prev_id = None
                            self.dis = 1.4
                        elif 1.3 < z_pos < 2.0:
                            self.prev_id = self.cur_id

                    if z_pos > self.dis:  # ar마커가 특정거리 이상일 때

                        x_pos = x_pos + self.add_x_pos
                        self.angle = x_pos * self.add_angle

                        if self.pass_third_ar and self.pass_fourth_ar == False:  # 3~4 사이
                            if z_pos >= 1.35:
                                self.angle = -20  # ar마커를 찾았고 세번째 ar마커를 통과했고 네번째 ar 마커를 통과하지 않았을 때
                        elif self.pass_second_ar and self.pass_third_ar == False:  # 2~3 사이
                            if z_pos >= 1.35:
                                self.angle = 20  # ar마커를 찾았고 두번째 ar마커를 통과했고 세번째 ar 마커를 통과하지 않았을 때
                            elif 0.95 <= z_pos < 1.35:
                                self.angle = 50


                    else:  # 특정거리 이내일 때 -> 스쳐지나가기
                        x_pos = x_pos + self.variable
                        self.angle = x_pos * 200

                self.drive(self.angle, self.speed)
            except Exception as e:
                rospy.logerr(f"Error in main loop: {str(e)}")

            rate.sleep()


if __name__ == '__main__':
    try:
        ar_driver = ARDriver()
        ar_driver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ARDriver node terminated.")