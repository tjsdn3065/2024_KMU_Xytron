#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 슈퍼카
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor
import heapq  # heapq 모듈을 임포트합니다. 우선순위 큐 기능을 제공합니다.

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [], []

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)


# parameter
MAX_T = 100.0  # 목표에 도달하기 위한 최대 허용 시간 (초)
MIN_T = 5.0    # 목표에 도달하기 위한 최소 허용 시간 (초)

class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # 시작 위치, 속도, 가속도, 목표 위치, 속도, 가속도와 경로 계획 시간을 기반으로 초기 계수 설정
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        # 5차 다항식의 나머지 계수(a3, a4, a5)를 계산하기 위한 선형 방정식 세우기
        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)  # 선형 방정식 해결하여 계수 찾기

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    # 특정 시간에서 위치 반환
    def calc_point(self, t):
        return self.a0 + self.a1 * t + self.a2 * t ** 2 + self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, dt):
    # 시작 조건과 목표 조건에서의 속도와 가속도 벡터 계산
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    # 최소 시간부터 최대 시간까지의 간격으로 5차 다항식 경로 시도
    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry=[],[],[]

        # 경로의 각 시점에 대한 위치, 속도, 가속도, 저크 계산
        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

    return rx, ry  # 조건을 충족하는 경로 반환

def convert_angle(sim_angle):
    standard_angle = (sim_angle + 90) % 360  # 시뮬레이션 각도를 90도 시계 방향으로 회전
    return standard_angle  # 라디안으로 변환

# P_ENTRY부터 P_END까지의 좌표 리스트 생성 함수
def generate_coordinates(p_entry, p_end, num_points=100):
    x1, y1 = p_entry
    x2, y2 = p_end
    
    x_coords = np.linspace(x1, x2, num_points)
    y_coords = np.linspace(y1, y2, num_points)
    
    rx = x_coords.tolist()
    ry = y_coords.tolist()
    
    return rx, ry

# A* 경로 탐색 후 P_ENTRY부터 P_END까지의 좌표 리스트 추가
def extend_path_with_parking(rx, ry, p_entry, p_end):
    parking_rx, parking_ry = generate_coordinates(p_entry, p_end)
    
    # 기존 경로에 주차 경로 추가
    rx.extend(parking_rx)
    ry.extend(parking_ry)
    
    return rx, ry

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    print("Start Planning")
    syaw = np.deg2rad(convert_angle(syaw))  # start yaw angle [rad]
    sv = 10.0  # start speed [m/s]
    sa = 0.1  # start accel [m/ss]
    gx = P_ENTRY[0]  # goal x position [m]
    gy = P_ENTRY[1]  # goal y position [m]
    gyaw = np.deg2rad(315.0)  # goal yaw angle [rad]
    gv = 10.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_jerk = 0.1  # max jerk [m/sss]
    rx, ry=quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, dt)
    rx, ry = extend_path_with_parking(rx, ry, P_ENTRY, P_END)
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    angle=20 # -20~20
    speed=50 # -50~50


    drive(angle,speed)  # 수정된 각도와 속도로 차량을 구동
