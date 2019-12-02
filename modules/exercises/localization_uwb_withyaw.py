# _*_ coding: utf-8 _*_
import sys
import math
import time
import numpy as np

from cyber_py import cyber
from modules.control.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import localization
from modules.localization.proto.localization_pb2 import pos
from modules.sensors.proto.nooploop_pb2 import TagFrame
from modules.sensors.proto.sensors_pb2 import Gyro 

class Exercise(object):
    def __init__(self, node):
        self.node = node
        self.alpha = 0.01
        self.maxTimes = 100
        self.x = np.array([0.725, 0.7]);
        # base station matrix
        self.base_x = np.array([0, 0, 2.5, 2.5])
        self.base_y = np.array([0, 1.32, 0, 1.32])
        
        self.xi = np.array([])
        self.yi = np.array([])
        # distance 
        self.di = np.array([])
        #self.di = np.array([8, 9.303, 0, 4.75])
        
        self.localization = localization()
        self.pos = pos()
        self.vel_head = 0
        self.yaw = 0
        self.direction = 0
        
        self.node.create_reader("/geek/gyro", Gyro, self.gyrocallback)
        self.node.create_reader("/geek/uwb/pose", TagFrame, self.tagcallback)
        self.node.create_reader("/chassis", Chassis, self.chassiscallback)
        self.writer = self.node.create_writer("/geek/uwb/localization", pos)
        self.fuseflag = 0
        self.v_x = 0
        self.head_x = 1
        self.head_y = 0
        self.v_y = 0
        self.direction_lag = 0
    def gyrocallback(self, gyro):
        if (0):
            #self.yaw -= 0.02 * gyro.gyro.z
            if (self.fuseflag == 1):
                self.yaw =  self.yaw + 0.1 * (self.vel_head - self.yaw)
            #self.yaw = self.yaw * 0.8 + 0.2 * self.vel_head
        else:
            gyro.gyro.z = -gyro.gyro.z * 0.02
            head_x_new = (self.head_x * math.cos(gyro.gyro.z) - self.head_y * math.sin(gyro.gyro.z)) 
            head_y_new = (self.head_x * math.sin(gyro.gyro.z) + self.head_y * math.cos(gyro.gyro.z))
            self.head_x = head_x_new
            self.head_y = head_y_new
            if (self.fuseflag == 1):
                lenth_v = math.sqrt(self.v_x * self.v_x + self.v_y * self.v_y)
                self.head_x += 0.3 * (self.v_x / lenth_v - self.head_x);
                self.head_y += 0.3 * (self.v_y / lenth_v - self.head_y);
            self.yaw = math.atan2(self.head_y, self.head_x)
        #self.yaw = self.vel_head
        if (self.yaw > 3.14159):
            self.yaw = self.yaw - 2 * 3.14159
        if (self.yaw < -3.14159):
            self.yaw = self.yaw + 2 * 3.14159
        
    def chassiscallback(self, chassis):
        if (chassis.speed > 0):
            self.direction = 1
        else:
            self.direction = -1


        if (abs(chassis.speed) < 0.1):
            self.direction = 0
        self.direction_lag = self.direction_lag * 0.5 + 0.5 * self.direction
        print self.direction

    def tagcallback(self, tag):
        self.di = np.array([])
        self.xi = np.array([])
        self.yi = np.array([])
        for k in range (0, 4):
            if tag.dis[k].distance == 1:
                continue
            self.di = np.append(self.di, tag.dis[k].distance)
            self.xi = np.append(self.xi, self.base_x[k])
            self.yi = np.append(self.yi, self.base_y[k])
            
        datas = np.array([[self.xi], [self.yi], [self.di]])
        datas = datas.T
        self.calcu_loss_fun(self.x, self.maxTimes, self.alpha, datas)
        v_x = tag.vel.x
        v_y = tag.vel.y
        #if (v_x * v_x + v_y * v_y) > 0.2:
        vel_head = math.atan2(v_y, v_x)
        self.fuseflag = 0
        if (self.direction_lag >= 0.95):
            if (v_x * v_x + v_y * v_y) > 0.01:
                self.fuseflag = 1
                self.v_x = -v_x
                self.v_y = -v_y
                self.vel_head = vel_head + 3.14159
                if (self.vel_head > 3.14159):
                    self.vel_head = self.vel_head - 2 * 3.14159
                if (self.vel_head < -3.14159):
                    self.vel_head = self.vel_head + 2 * 3.14159
        if 0 :
            if (v_x * v_x + v_y * v_y) > 0.04:
                self.fuseflag = 1
                self.v_y = v_y
                self.vel_head = vel_head + 3.14159
                if (self.vel_head > 3.14159):
                     self.vel_head = self.vel_head - 2 * 3.14159
                if (self.vel_head < -3.14159):
                     self.vel_head = self.vel_head + 2 * 3.14159
        if self.direction == 0:
            self.vel_head = self.vel_head
            
            self.fuseflag = 0
        #print self.vel_head
            

        
        
    # 定义函数f(x)
    def problem(self, x, data):
        #e = 2.71828182845904590
        return (x[0] - data[0,0]) ** 2 +  (x[1] - data[0,1])**2 - data[0,2]**2  

    #定义损失函数
    def loss_fun(self, x, datas):
        sum_err = 0;
        for data in datas:
          sum_err += 0.5 * (self.problem(x, data) - 0)**2
        return sum_err

    #计算损失函数的斜率
    def slope_fx(self, x, datas):
        d = 0.01;
        delta1  = np.array([d, 0]);
        delta2  = np.array([0, d]);
        J1 = (self.loss_fun(x+delta1, datas) - self.loss_fun(x-delta1, datas)) / (2.0*d)
        J2 = (self.loss_fun(x+delta2, datas) - self.loss_fun(x-delta2, datas)) / (2.0*d)
        return [J1, J2]

    #代入f(x)，计算数值
    def calcu_loss_fun(self, x, maxTimes, alpha, datas):
        for i in range(maxTimes):
                ret = self.slope_fx(x, datas)
                x1 = x[0] - ret[0]*alpha;
                x2 = x[1] - ret[1]*alpha;	
                x = np.array([x1, x2])
        if self.loss_fun(x, datas) > 1000:
            return
        #print 'times %d, x: %.13f,y: %.13f f(x): %.13f' % (i, x[0],x[1], self.loss_fun(x, datas))
        
        self.localization.apriltag0.x = x[0]
        self.localization.apriltag0.y = -1
        self.localization.apriltag0.z = x[1]
        self.localization.apriltag0.yaw = -1
        self.localization.apriltag1.x = x[0]
        self.localization.apriltag1.y = -1
        self.localization.apriltag1.z = x[1]
        self.localization.apriltag1.yaw = -1
        self.localization.apriltag.x = x[0]
        self.localization.apriltag.y = -1
        self.localization.apriltag.z = x[1]
        self.localization.apriltag.yaw = -1
        self.localization.predict.x = x[0]
        self.localization.predict.y = -1
        self.localization.predict.z = x[1]
        self.localization.predict.yaw = -1
        self.pos.x = x[0]
        self.pos.y = x[1]
        #self.pos.x = self.head_x
        #self.pos.y = self.head_y
        self.pos.z = self.vel_head
        self.pos.yaw = self.yaw
        self.writer.write(self.pos)


if __name__ == '__main__':
    cyber.init()

    exercise_node = cyber.Node("localization_node")
    
    exercise = Exercise(exercise_node)
    exercise_node.spin()
    cyber.shutdown()



