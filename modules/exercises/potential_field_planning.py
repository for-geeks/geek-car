"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import numpy as np

from cyber_py import cyber
from modules.planning.proto.planning_pb2 import PlanningInfo
from modules.planning.proto.planning_pb2 import Trajectory
from modules.planning.proto.planning_pb2 import Point

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 2.0  # potential area width [m]

show_animation = False


def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    # if show_animation:
    #     draw_heatmap(pmap)
    #     plt.plot(ix, iy, "*k")
    #     plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        # if show_animation:
        #     plt.plot(ix, iy, ".r")
        #     plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    # plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


class planning(object):

    def __init__(self, node):
        self.node = node
        self.node.create_reader(
            "/planning/target", PlanningInfo, self.callback)
        self.writer = self.node.create_writer("/planning/a_star", Trajectory)

    def callback(self, data):
        sx = data.start_point.x  # start x position [m]
        sy = data.start_point.y  # start y positon [m]
        gx = data.end_point.x  # goal x position [m]
        gy = data.end_point.y  # goal y position [m]
        grid_size = 0.1  # potential grid size [m]
        robot_radius = 0.125  # robot radius [m]

        print('start point,{} goal point,{}'.format(
            data.start_point, data.end_point))

        ox = []
        oy = []

        for obstacle in data.obs_points:
            ox.append(obstacle.x)
            oy.append(obstacle.y)

        print('obstacle information:{} '.format(data.obs_points))

        # ox = [0.1]  # obstacle x position list [m]
        # oy = [0.2]  # obstacle y position list [m]

        # if show_animation:
        # plt.grid(True)
        # plt.axis("equal")

        # path generation
        rx, ry = potential_field_planning(
            sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

        print('rx,{}, ry.{}'.format(rx, ry))
        self.planning_path = Trajectory()
        if not rx:
            print("Failed to find a path")
        else:
            point = Point()
            for i, _ in enumerate(rx):
                point.x = rx[i]
                point.y = ry[i]
                self.planning_path.point.append(point)
            print('trajectory,{}'.format(self.planning_path))
            self.write_to_channel()

    def write_to_channel(self):
        if not cyber.is_shutdown():
            self.writer.write(self.planning_path)


def main():
    print("potential_field_planning start")

    cyber.init()
    planning_node = cyber.Node("planning")
    _ = planning(planning_node)

    planning_node.spin()
    cyber.shutdown()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
