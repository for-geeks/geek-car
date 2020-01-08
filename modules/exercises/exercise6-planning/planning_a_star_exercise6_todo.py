#!/usr/bin/env python
# coding:utf-8
import os
import sys
import time
import signal
from cyber_py import cyber
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point

from modules.localization.proto.localization_pb2 import localization
from modules.localization.proto.localization_pb2 import pos

import cv2
import numpy as np

point_xy = Point()

maps = cv2.imread("maps1.jpeg", cv2.IMREAD_GRAYSCALE)  # 读取地图图像，灰度读入。灰度为0表示障碍物
maps_size = np.array(maps)  # 获取图像行和列大小
hight = maps_size.shape[0]  # 行数->y
width = maps_size.shape[1]  # 列数->x
scale = 144.9

class planning(object):

	def __init__(self, node):

		self.node = node
                self.start_x = 0
                self.start_y = 0
                self.goal_x = 0
                self.goal_y = 0
                self.node.create_reader("/geek/uwb/localization", pos, self.localizationcallback)
                self.node.create_reader("/planning/mission_point", Point, self.missioncallback)
                self.writer = self.node.create_writer("/planning/global_trajectory", Trajectory)

                signal.signal(signal.SIGINT, self.sigint_handler)
                signal.signal(signal.SIGHUP, self.sigint_handler)
                signal.signal(signal.SIGTERM, self.sigint_handler)
                self.is_sigint_up = False
                while True:
                    time.sleep(0.05)
                    if self.is_sigint_up:
                        print("Exit!")
                        self.is_sigint_up = False
                        sys.exit()

        def sigint_handler(self, signum, frame):
            self.is_sigint_up = True
            print("catch interrupt signal!")

        def localizationcallback(self, pos):
            
            self.start_x = int(pos.x*scale)
            self.start_y = int(pos.y*scale)

        def missioncallback(self, Point):

            self.goal_x = int(Point.x)
            self.goal_y = int(Point.y)

            pathList = self.start(self.start_x, self.start_y, self.goal_x, self.goal_y)

            self.planning_path = Trajectory()
            if not pathList:
                print("Failed to find a path")
            else:
                for path_point in pathList:
                    point_xy.x = path_point[0]
                    point_xy.y = path_point[1]

                    self.planning_path.point.append(point_xy)

            if not cyber.is_shutdown() and self.planning_path:
                self.writer.write(self.planning_path)

        def start(self, start_x, start_y, goal_x, goal_y):

            if not os.path.exists('global.txt'):
                f = open("global.txt", 'w')
                f.close()

            plan_path = []
            maps_size = np.array(maps)  # 获取图像行和列大小
            height = maps_size.shape[0]  # 行数->y
            width = maps_size.shape[1]  # 列数->x
    
            star = {'position': (start_x, start_y), 'cost': 700, 'parent': (start_x, start_y)}  # 起点
            end = {'position': (goal_x, goal_y), 'cost': 0, 'parent': (goal_x, goal_y)}  # 终点

            print('start_point:', [start_x, start_y])
            print('end_point:', [goal_x, goal_y])

            if maps[start_y, start_x] == 255:
                print("error: 非法起点")
                return []

            if maps[goal_y, goal_x] == 255:
                print("error: 非法终点")
                return []

            openlist = []  # open列表，存储可能路径
            closelist = [star]  # close列表，已走过路径
            step_size = 10  # 搜索步长
            step_size_scan = 1
            safe_size = step_size
            # 步长太小，搜索速度就太慢。步长太大，可能直接跳过障碍，得到错误的路径
            # 步长大小要大于图像中最小障碍物宽度
            time_start = time.time()

            while 1:
                s_point = closelist[-1]['position']  # 获取close列表最后一个点位置，S点
                
                add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0], [-step_size, step_size],
                       [step_size, -step_size], [step_size, step_size], [-step_size, -step_size])  # 可能运动的四个方向增量

                add_scan = ([0, step_size_scan], [0, -step_size_scan], [step_size_scan, 0], [-step_size_scan, 0],
                            [-step_size_scan, step_size_scan], [step_size_scan, -step_size_scan], [step_size_scan, step_size_scan], [-step_size_scan, -step_size_scan])
                
                lane_point = []
                current_point = [s_point[0], s_point[1]]

                n = 1

                if maps[s_point[1], s_point[0]] >= 120 and maps[s_point[1], s_point[0]] <= 140:
                    lane_point = [s_point[0], s_point[1]]

                while lane_point == []:
                    for i in range(len(add_scan)):
                        x = current_point[0] + n*add_scan[i][0]  # 检索超出图像大小范围则跳过
                        if x < 0 or x >= width:
                            continue
                        y = current_point[1] + n*add_scan[i][1]
                        if y < 0 or y >= height:  # 检索超出图像大小范围则跳过
                            continue
                        
                        if maps[y, x] >= 120 and maps[y, x] <= 140:
                            lane_point = [x, y]
                            break

                    n += 1

                    if lane_point != []:
                        break
                
                for i in range(len(add)):
                    x = s_point[0] + add[i][0]  # 检索超出图像大小范围则跳过
                    if x < 0 or x >= width:
                        continue
                    y = s_point[1] + add[i][1]
                    if y < 0 or y >= height:  # 检索超出图像大小范围则跳过
                        continue

                    ##构建花费g和h
                    ##TODO

                    G = abs(x - star['position'][0]) + abs(y - star['position'][1])  # 计算代价
                    H = abs(x - end['position'][0]) + abs(y - end['position'][1])  # 计算代价

                    ##TODO

                    I = abs(x - lane_point[0]) + abs(y - lane_point[1])
                    
                    if (maps[y, x] >= 120 and maps[y, x] <= 140) or ((x - end['position'][0])**2 + (y - end['position'][1])**2)**0.5 < safe_size:
                        G_I = 0
                    else:
                        G_I = I * 30

                    F = G + H + G_I

                    if ((x - end['position'][0]) ** 2 + (y - end['position'][1]) ** 2) ** 0.5 <= step_size:  # 当逐渐靠近终点时，搜索的步长变小
                        step_size = 1

                    ##构建a star算法
                    ##TODO
                    addpoint = {'position': (x, y), 'cost': F, 'parent': s_point}  # 更新位置
                    count = 0
                    for i in openlist:
                        if i['position'] == addpoint['position']:
                            count += 1
                    for i in closelist:
                        if i['position'] == addpoint['position']:
                            count += 1
                    if count == 0:  # 新增点不在open和close列表中
                        if maps[int(y), int(x)] != 255:  # 非障碍物
                            openlist.append(addpoint)

                t_point = {'position': (50, 50), 'cost': 10000, 'parent': (50, 50)}
                for j in range(len(openlist)):  # 寻找代价最小点
                    if openlist[j]['cost'] < t_point['cost']:
                        t_point = openlist[j]
                for j in range(len(openlist)):  # 在open列表中删除t点
                    if t_point == openlist[j]:
                        openlist.pop(j)
                        break
                closelist.append(t_point)  # 在close列表中加入t点

                # cv2.circle(informap,t_point['位置'],1,(200,0,0),-1)
                if t_point['position'] == end['position']:  # 找到终点！！
                    print("找到终点")
                    break

                ##TODO

                if len(closelist) > ((1300*80)/step_size):
                    print("Error: can not find the goal!")
                    break

            # 逆向搜索找到路径
            road = []
            road.append(closelist[-1])
            point = road[-1]
            k = 0
    
            while 1:
                print("3")
                for i in closelist:
                    if i['position'] == point['parent']:  # 找到父节点
                        point = i
                        road.append(point)
                if point == star:
                    print("路径搜索完成")
                    break

            for i in road:  # 画出规划路径
                plan_path.append(i['position'])

            time_end = time.time()

            print('totally cost', time_end - time_start)

            f = open('global.txt', 'w')
            f.write(str(plan_path))
            f.close()

            return plan_path


if __name__ == '__main__':

    cyber.init()
    cyber_node = cyber.Node("planning")
    exercise = planning(cyber_node)
    
    cyber_node.spin()
    cyber.shutdown()
