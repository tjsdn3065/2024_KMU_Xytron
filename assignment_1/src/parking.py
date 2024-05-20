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


# 그리드 상의 노드를 나타내는 클래스 정의
class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x  # 노드의 x 좌표
        self.y = y  # 노드의 y 좌표
        self.cost = cost  # 시작 노드부터 현재 노드까지의 경로 비용
        self.pind = pind  # 부모 노드의 인덱스

# 알고리즘 파라미터를 저장하는 클래스 정의
class Para:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, motion):
        self.minx = minx  # 그리드의 최소 x 좌표
        self.miny = miny  # 그리드의 최소 y 좌표
        self.maxx = maxx  # 그리드의 최대 x 좌표
        self.maxy = maxy  # 그리드의 최대 y 좌표
        self.xw = xw      # x축의 너비 (최대 x - 최소 x)
        self.yw = yw      # y축의 높이 (최대 y - 최소 y)
        self.motion = motion  # 가능한 이동 방향 (벡터) 리스트

# A* 경로 탐색 함수
def astar_planning(sx, sy, gx, gy):
    # 시작 노드와 목표 노드를 초기화
    n_start = Node(sx,sy, 0.0, -1)  # round() 입력된 숫자를 가장 가까운 정수로 반올림
    n_goal = Node(gx,gy, 0.0, -1)

    # 파라미터 객체 생성, 예제 그리드 크기 사용
    P = Para(0, 0, 1200, 850, 1200, 850, get_motion())

    # 열린 집합과 닫힌 집합을 사전으로 초기화
    open_set, closed_set = dict(), dict() # dict()는 파이썬에서 사전(dictionary) 객체를 생성하는 내장 함수입니다. 사전은 키(key)와 값(value)의 쌍을 저장하는 자료 구조이다. 해쉬테이블
    open_set[calc_index(n_start, P)] = n_start

    # 우선순위 큐 초기화 및 시작 노드 추가
    q_priority = []
    heapq.heappush(q_priority, (fvalue(n_start, n_goal), calc_index(n_start, P)))
    # heapq.heappush(heap, item) 함수는 item을 heap에 추가하면서 힙의 속성을 유지합니다. 여기서 item은 튜플 (fvalue(n_start, n_goal), calc_index(n_start, P))입니다.

    # 주 탐색 루프
    while True:
        if not open_set: # 열린 목록이 비게 된다면 탐색 중단
            break

        # 현재 노드를 우선순위 큐에서 꺼냄
        _, ind = heapq.heappop(q_priority)  # 큐에서 꺼내면 가장 작은 비용을 가진 노드의 비용과 인덱스가 추출
        n_curr = open_set[ind]              # 그 인덱스를 열린목록에 넣기
        closed_set[ind] = n_curr            # 현재 노드를 닫힌 목록에 넣기
        open_set.pop(ind)                   # 열린 목록에서 제거
        if calc_index(n_curr, P) == calc_index(n_goal, P):
            break  # 목표 노드 도달 시 종료


        # 가능한 모든 이동 방향에 대해 루프
        for i in range(len(P.motion)):      # 인접한 노드의 비용 계산, 현재 노드를 부모 노드로 지정
            node = Node(n_curr.x + P.motion[i][0],
                        n_curr.y + P.motion[i][1],
                        n_curr.cost + u_cost(P.motion[i]), ind)

            n_ind = calc_index(node, P)     # 인접한 노드의 인덱스
            if n_ind not in closed_set:     # 닫힌 목록에 없는 노드이고
                if n_ind in open_set:       
                    if open_set[n_ind].cost > node.cost:    # 인접한 노드가 이미 기존의 열린 목록에 있다면 현재 노드를 기준으로 해당 인접한 노드까지 이동할 때 비용이 낮아지는 지 확인
                        open_set[n_ind].cost = node.cost    # 비용이 더 낮아지지 않으면 아무것도 하지 않는다
                        open_set[n_ind].pind = ind          # 만약 현재 노드를 통해 해당 인접 노드까지 이동하는데 비용이 더 낮게 니온다면
                else:                                       # 해당 인접 노드의 부모 노드를 현재 노드로 바꾼다 그리고 해당 인접 노드의 비용을 다시 계산
                    open_set[n_ind] = node
                    heapq.heappush(q_priority,
                                   (fvalue(node, n_goal), calc_index(node, P)))

    # 최종 경로 추출
    pathx, pathy = extract_path(closed_set, n_start, n_goal, P)

    return pathx, pathy

# 이동 비용 계산 함수
def u_cost(u):
    return math.hypot(u[0], u[1])  # 유클리드 거리를 이용한 비용 계산

# 휴리스틱 비용 함수 F값 계산
def fvalue(node, n_goal):
    return node.cost + h(node, n_goal)  # G값과 H값의 합

# 휴리스틱 거리 측정값
def h(node, n_goal):
    return math.hypot(node.x - n_goal.x, node.y - n_goal.y) # 맨하탄 거리

# 노드 인덱스 계산 함수
def calc_index(node, P):
    return (node.y - P.miny) * P.xw + (node.x - P.minx) # 2차원 배열을 1차원으로

# 가능한 이동 방향 반환 함수
def get_motion():
    return [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
            #  서       남서      남      남동     동       북동      북        북서

# 경로 추출 함수
def extract_path(closed_set, n_start, n_goal, P):
    pathx, pathy = [n_goal.x], [n_goal.y]
    n_ind = calc_index(n_goal, P)

    # 경로를 거슬러 올라가며 좌표를 추출
    while n_ind != calc_index(n_start, P):
        node = closed_set[n_ind]
        pathx.append(node.x)
        pathy.append(node.y)
        n_ind = node.pind

    # 좌표를 실제 스케일로 조정
    pathx = [x for x in reversed(pathx)]
    pathy = [y for y in reversed(pathy)]

    return pathx, pathy


#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    print("Start Planning")
    # A* 알고리즘 실행
    rx, ry = astar_planning(sx, sy, 1036, 162)
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    angle = 50 # -50 ~ 50
    speed = 50 # -50 ~ 50
    
    drive(angle, speed)

