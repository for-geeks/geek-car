# coding:utf-8
import time
import random
from cyber_py import cyber
from modules.planning.proto.planning_pb2 import PlanningInfo
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point

point_xy = Point()

send_flag = 0

map_w = 20
map_h = 20
map_x_min = -10
map_x_max = 10
map_y_min = 0
map_y_max = 20


class Point:
    """
    表示一个点
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __str__(self):
        return '(x:{}, y:{})'.format(self.x, self.y)


class AStar:
    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            # self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 10  # 计算h值
            # self.h = int((pow(abs(endPoint.x - point.x), 2) + pow(abs(endPoint.y - point.y), 2)) ** 0.5)# 计算h值,使用欧式距离
            self.h = pow(abs(endPoint.x - point.x), 2) + \
                pow(abs(endPoint.y - point.y), 2)  # 计算h值

    def __init__(self, startPoint, endPoint):
        """
        构造AStar算法的启动条件
        :param startPoint: Point类型的寻路起点
        :param endPoint: Point类型的寻路终点
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 起点终点
        self.startPoint = startPoint
        self.endPoint = endPoint

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):

        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):

        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY):
        """
        搜索节点周围的点, 更新openlist, 重新计算G值、设置father(如有需要)
        :param minF:
        :param offsetX:
        :param offsetY:
        :return:
        """
        # 越界检测
        if minF.point.x + offsetX < map_x_min or minF.point.x + offsetX > map_x_max - 1 or minF.point.y + offsetY < map_y_min or minF.point.y + offsetY > map_y_max - 1:
            return
        # 如果是障碍，就忽略
        for obs in obslist:
            if obs[0] == minF.point.x + offsetX and obs[1] == minF.point.y + offsetY:
                return

        # 如果在关闭表中，就忽略
        if self.pointInCloseList(Point(minF.point.x + offsetX, minF.point.y + offsetY)):
            return
        # 设置单位花费
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        # 如果不在openList中，就把它加入openlist
        currentNode = self.pointInOpenList(
            Point(minF.point.x + offsetX, minF.point.y + offsetY))
        if not currentNode:
            currentNode = AStar.Node(Point(minF.point.x + offsetX, minF.point.y + offsetY), self.endPoint,
                                     g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF

    def expansion(self, offset=0):
        """
        地图障碍物膨胀
        :param offset: 膨胀次数
        """
        obslist_old = []

        for point in obslist:
            obslist_old.append(point)

        for i in range(offset):
            for xy in obslist_old:
                offset = 1
                points = [[-offset, offset], [0, offset], [offset, offset], [-offset, 0],
                          [offset, 0], [-offset, -offset], [0, -offset], [offset, -offset]]
                for point in points:
                    if map_x_min <= xy[0] + point[0] < map_x_max and map_y_min <= xy[1] + point[1] < map_y_max:
                        obslist.append([xy[0] + point[0], xy[1] + point[1]])

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        # 主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()

            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)

            # 判断这个节点的上下左右节点
            self.searchNear(minF, -1, 1)
            self.searchNear(minF, 0, 1)
            self.searchNear(minF, 1, 1)
            self.searchNear(minF, -1, 0)
            self.searchNear(minF, 1, 0)
            self.searchNear(minF, -1, -1)
            self.searchNear(minF, 0, -1)
            self.searchNear(minF, 1, -1)

            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None


def planning_reader():
    test_node = cyber.Node("planning_a_star_py")
    test_node.create_reader("/planning/target", PlanningInfo, callback)
    return test_node


def callback(data):

    global obslist, map_w, map_h, map_x_min, map_x_max, map_y_min, map_y_max, planning_path
    obslist = []

    planning_path = Trajectory()

    pStart_x = int(20 * data.start_point.x)
    pStart_y = int(20 * data.start_point.y)
    pEnd_x = int(20 * data.end_point.x)
    pEnd_y = int(20 * data.end_point.y)
    pStart = Point(pStart_x, pStart_y)
    pEnd = Point(pEnd_x, pEnd_y)

    for point in data.obs_points:
        obslist.append([int(20 * point.x), int(20 * point.y)])

    for i in range(0):
        obslist.append([random.uniform(2, pEnd.y), random.uniform(2, pEnd.y)])

    time_start = time.time()
    # 创建AStar对象,并设置起点终点
    aStar = AStar(pStart, pEnd)
    aStar.expansion(offset=1)

    # 开始寻路
    pathList = aStar.start()
    time_end = time.time()
    print('totally cost', time_end - time_start)

    global planning_points

    for path_point in pathList:
        point_xy.x = path_point.x * 0.05
        point_xy.y = path_point.y * 0.05

        planning_path.point.append(point_xy)

    global send_flag
    send_flag = 1


def planning_router(node):
    g_count = 1
    writer = node.create_writer("/planning/a_star", Trajectory)
    while not cyber.is_shutdown():
        global send_flag
        if send_flag:
            g_count = g_count + 1
            global line_msg
            writer.write(planning_path)
            send_flag = 0


def test():
    global obslist, map_w, map_h, map_x_min, map_x_max, map_y_min, map_y_max

    pStart_x = 0
    pStart_y = 0
    pEnd_x = -5
    pEnd_y = 20 - 1
    pStart = Point(pStart_x, pStart_y)
    pEnd = Point(pEnd_x, pEnd_y)

    obslist = [[0, 3], [-4, 15]]

    for i in range(0):
        obslist.append([random.uniform(2, pEnd.y), random.uniform(2, pEnd.y)])

    # 创建AStar对象,并设置起点终点
    aStar = AStar(pStart, pEnd)
    aStar.expansion(offset=1)

    # 开始寻路
    pathList = aStar.start()

    for point in pathList:
        print('(' + str(point.x) + ',' + str(point.y) + ')')

    print(pathList)


if __name__ == '__main__':

    cyber.init()
    cyber_node = planning_reader()
    planning_router(cyber_node)

    cyber_node.spin()
    cyber.shutdown()

    # test()
