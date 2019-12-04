#!/usr/bin/env python

import math
import time
import numpy as np
from cyber_py import cyber
from modules.planning.proto.planning_pb2 import PlanningInfo
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point

point_xy = Point()

yawrate_old = 0


class Config(object):
    """
    用来仿真的参数，
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.5  # [m/s]  # 最大速度
        # self.min_speed = 0  # [m/s]  # 最小速度，设置为可以倒车
        self.min_speed = 0  # [m/s]  # 最小速度，设置为不倒车
        self.max_yawrate = 60.0 * math.pi / 180.0  # [rad/s]  # 最大角速度s
        self.max_accel = 1.1  # [m/ss]  # 最大加速度
        self.max_dyawrate = 600.0 * math.pi / 180.0  # [rad/ss]  # 最大角加速度
        self.v_reso = 0.25  # [m/s]，速度分辨率
        self.yawrate_reso = 10 * math.pi / 180.0  # [rad/s]，角速度分辨率
        self.dt = 0.1  # [s]  # 采样周期
        self.predict_time = 3  # [s]  # 向前预估三秒
        self.to_goal_cost_gain = 6  # 目标代价增益
        self.speed_cost_gain = 10  # 速度代价增益
        self.obstacle_cost_gain = 1  # 障碍物代价增益
        self.yawrate_cost_gain = 8  # 角速度代价增益
        self.robot_radius = 0.2  # [m]  # 机器人半径


def motion(x, u, dt):
    """
    :param x: 位置参数，在此叫做位置空间
    :param u: 速度和加速度，在此叫做速度空间
    :param dt: 采样时间
    :return:
    """
    # TODO
    x[0] += u[0] * math.cos(x[2]) * dt  # x方向位移
    x[1] += u[0] * math.sin(x[2]) * dt  # y
    x[2] += u[1] * dt  # 航向角
    x[3] = u[0]  # 速度v
    x[4] = u[1]  # 角速度w

    return x


def calc_dynamic_window(x, config):
    """
    位置空间集合
    :param x:当前位置空间,符号参考硕士论文
    :param config:
    :return:目前是两个速度的交集，还差一个
    """

    # 车辆能够达到的最大最小速度
    vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # 一个采样周期能够变化的最大最小速度
    vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    # 求出两个速度集合的交集
    vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
          max(vs[2], vd[2]), min(vs[3], vd[3])]

    return vr


def calc_trajectory(x_init, v, w, config):
    """
    预测3秒内的轨迹
    :param x_init:位置空间
    :param v:速度
    :param w:角速度
    :param config:
    :return: 每一次采样更新的轨迹，位置空间垂直堆叠
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        trajectory = np.vstack((trajectory, x))  # 垂直堆叠，vertical
        time += config.dt

    return trajectory


def calc_to_goal_cost(trajectory, goal, config):
    """
    计算轨迹到目标点的代价
    :param trajectory:轨迹搜索空间
    :param goal:
    :param config:
    :return: 轨迹到目标点欧式距离
    """
    # calc to goal cost. It is 2D norm.

    # TODO
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    goal_dis = math.sqrt(dx ** 2 + dy ** 2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def calc_obstacle_cost(traj, ob, config):
    """
    计算预测轨迹和障碍物的最小距离，dist(v,w)
    :param traj:
    :param ob:
    :param config:
    :return:
    """
    # calc obstacle cost inf: collision, 0:free

    # TODO

    skip_n = 2  # 省时

    minr = float("inf")  # 距离初始化为无穷大

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox

            dy = traj[ii, 1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # 越小越好


def calc_speed_cost(traj, config):
    """
    计算预测速度与最大速度差距
    :param traj:
    :param config:
    :return:
    """
    # TODO

    speed_cost = config.max_speed - traj[-1, 3]

    return speed_cost


def calc_final_input(x, u, vr, config, goal, ob):
    """
    计算采样空间的评价函数，选择最合适的那一个作为最终输入
    :param x:位置空间
    :param u:速度空间
    :param vr:速度空间交集
    :param config:
    :param goal:目标位置
    :param ob:障碍物
    :return:
    """
    global yawrate_old

    x_init = x[:]
    min_cost = 10000.0
    min_u = u

    best_trajectory = np.array([x])

    # v,生成一系列速度，w，生成一系列角速度
    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):

            trajectory = calc_trajectory(x_init, v, w, config)

            # calc cost
            to_goal_cost = config.to_goal_cost_gain * \
                calc_to_goal_cost(trajectory, goal, config)
            speed_cost = config.speed_cost_gain * \
                calc_speed_cost(trajectory, config)
            ob_cost = config.obstacle_cost_gain * \
                calc_obstacle_cost(trajectory, ob, config)

            # 用于稳定规划路径，减少跳动
            yawrate_cost = config.yawrate_cost_gain * abs(w - yawrate_old)

            # 评价函数多种多样，看自己选择
            # 本文构造的是越小越好
            final_cost = to_goal_cost + speed_cost + ob_cost + yawrate_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
                best_trajectory = trajectory

    yawrate_old = min_u[1]

    return min_u, best_trajectory


def dwa_control(x, u, config, goal, ob):
    """
    调用前面的几个函数，生成最合适的速度空间和轨迹搜索空间
    :param x:
    :param u:
    :param config:
    :param goal:
    :param ob:
    :return:
    """

    vr = calc_dynamic_window(x, config)

    u, trajectory = calc_final_input(x, u, vr, config, goal, ob)

    return u, trajectory


class planning(object):

    def __init__(self, node):
        self.node = node
        self.node.create_reader(
            "/planning/target", PlanningInfo, self.callback)
        self.writer = self.node.create_writer(
            "/planning/trajectory", Trajectory)

    def callback(self, data):
        global obslist, planning_path, best_trajectory
        obslist = []

        print(" start!!")

        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x = np.array([0.0, 0.0, math.pi / 2.0, 0.3, 0.0])
        # goal position [x(m), y(m)]
        goal = np.array([data.end_point.x, data.end_point.y])

        # obstacles [x(m) y(m), ....]
        for point in data.obs_points:
            obslist.append([point.x, point.y])

        ob = np.matrix(obslist)

        # input [forward speed, yawrate]
        u = np.array([0.3, 0.0])
        config = Config()

        best_trajectory = np.array(x)

        u, best_trajectory = dwa_control(x, u, config, goal, ob)

        x = motion(x, u, config.dt)

        self.planning_path = Trajectory()

        if not best_trajectory.any():
            print("Failed to find a path")
        else:
            for path_point in best_trajectory:
                point_xy.x = path_point[0]
                point_xy.y = path_point[1]

                self.planning_path.point.append(point_xy)

        if not cyber.is_shutdown() and self.planning_path:
            self.writer.write(self.planning_path)

        print("Done")


if __name__ == '__main__':
    cyber.init()
    cyber_node = cyber.Node("planning")
    exercise = planning(cyber_node)

    cyber_node.spin()
    cyber.shutdown()
