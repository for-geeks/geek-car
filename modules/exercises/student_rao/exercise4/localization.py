# -*- coding: utf-8 -*-

import sys

sys.path.append("../")
from cyber_py import cyber
from modules.localization.proto.localization_pb2 import Tags
from modules.localization.proto.localization_pb2 import localization
from modules.localization.proto.localization_pb2 import pos
from modules.sensors.proto.sensors_pb2 import Point
from modules.sensors.proto.sensors_pb2 import Pose
from modules.control.proto.chassis_pb2 import Chassis
import time
import sys
import numpy as np
import math

class kalman_filter(object):
    def __init__(self):
	self.kalman_pos = pos()
	self.kalman_pos.x = 0
	self.kalman_pos.y = 0
	self.kalman_pos.z = 0
	self.kalman_pos.yaw = 0
	self.obstacle_pos = pos()
	self.P = self.covariance(1000, 100)   
	self.A = np.array([[1, 0], [0, 1]])
	self.H = np.identity(2)
	self.R = self.covariance(1000, 100)
	self.K = self.H
	self.X = np.array([[self.kalman_pos.x], [self.kalman_pos.z]])	

    def localization_with_kalman(self, speed, yaw):
	self.kalman_prediction(speed, yaw);
	self.P = np.diag(np.diag(self.A.dot(self.P).dot(self.A.T)))
	S = self.H.dot(self.P).dot(self.H.T) + self.R
	self.K = self.P.dot(self.H).dot(np.linalg.inv(S))
	print ".>>>>>>>>>>>>>>>>>>>>>>>>"
	print self.K
	#self.kalman_update();
	pass

    def kalman_prediction(self, speed, yaw):
	
	#X = X.T
	B = np.array([[speed * 0.05 * math.sin(yaw)], [speed * 0.05 * math.cos(yaw)]])
	self.X = self.A.dot(self.X) + B

    def kalman_update(self, data):
	Y = np.array([[data.x], [data.z]])
	self.X = self.X + 0.1 * self.K.dot(Y - self.X)
	self.kalman_pos.x = self.X[0]
	self.kalman_pos.z = self.X[1]


    def covariance(self, x, z):
	cov_matrix = np.array([[x * x, x * z], [z * x, z * z]])
	return np.diag(np.diag(cov_matrix))



class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Tags()

	self.localization = localization()
	self.position_0 = pos()
	self.position_1 = pos()
	self.pos = pos()
	self.kalman_pos = pos()
	self.kalman_pos.x = 0
	self.kalman_pos.y = 0
	self.kalman_pos.z = 0
	self.kalman_pos.yaw = 0
	self.obstacle_pos = pos()
	self.marker_pos = {0: [1.0, 0], 1: [2.0, 0]}
        self.node.create_reader("/localization/tag", Tags, self.callback) 
        self.node.create_reader("/realsense/pose", Pose, self.posecallback) 
        self.node.create_reader("/chassis", Chassis, self.Chassiscallback) 
        self.writer = self.node.create_writer("/localization", localization)
	self.writer_kalman = self.node.create_writer("/localization/kalman_filter", pos)
	self.writer_obstacle = self.node.create_writer("/localization/obstacle", pos)
	self.start_yaw = 0
	self.init_flag = 0;
	self.obstacle_flag = 0
	self.speed = 0
	self.yaw = 0
	self.kalman = kalman_filter()
	while not cyber.is_shutdown():
	    time.sleep(0.05)
	    self.localization_with_odometer_calculation();
	    #self.localization_with_kalman();
	    self.kalman.localization_with_kalman(self.speed, self.yaw)
	    self.writer.write(self.localization)
	    self.writer_kalman.write(self.kalman.kalman_pos)
	    if self.obstacle_flag:
	        self.obstacle_flag = 0
	    else:
		self.obstacle_pos.x = -1
		self.obstacle_pos.y = -1
		self.obstacle_pos.z = -1
		self.obstacle_pos.yaw = -1
	    self.writer_obstacle.write(self.kalman.obstacle_pos)
	    print self.localization

    def callback(self,data):
        self.get_global_pos_by_apriltag(data)

    def Chassiscallback(self, Chassis):
	self.speed = Chassis.speed

    def posecallback(self, Pose):
	if self.init_flag == 1:

            q1 = Pose.rotation.x
            q2 = Pose.rotation.y
            q3 = Pose.rotation.z
            q0 = Pose.rotation.w
            yaw = math.atan2((q1 * q2 - q0 * q3) * 2, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) - self.start_yaw
	    self.yaw = yaw;
	else:
	    q1 = Pose.rotation.x
            q2 = Pose.rotation.y
            q3 = Pose.rotation.z
            q0 = Pose.rotation.w
            start_yaw = math.atan2((q1 * q2 - q0 * q3) * 2, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) - self.start_yaw
	    self.init_flag = 1
	    self.localization.predict.z = 0
	    self.localization.predict.y = 0
	    self.localization.predict.x = 0
	    self.localization.predict.yaw = 0
	    #start_yaw
	
    def get_global_pos_by_apriltag(self, data): 
        pitch = -15 * 3.14159265358979 / 180
	print math.sin(pitch * 2)
	for i in range(0, len(data.tag)):
	    if (data.tag[i].id == 0):
		Rt = np.asarray(data.tag[i].pose.r.element)  
		T = np.asarray(data.tag[i].pose.t.element)
		T.shape=(1,3)
		T = T.T
		Rt = Rt.reshape((3,3),order='C')
		Rt_inv = np.linalg.inv(Rt)
		print "!=================================="
		print Rt_inv
		print T
		print np.matmul(Rt_inv, T)#np.multiply(Rt_inv, T)
		Tre = np.matmul(Rt_inv, T)
	       
		r21 = data.tag[i].pose.r.element[2 * 3 + 1]
  		r22 = data.tag[i].pose.r.element[2 * 3 + 2]
  		sy = math.sqrt(r21 * r21 + r22 * r22)
  		theta_y = math.atan2(-data.tag[i].pose.r.element[2 * 3 + 0], sy)

		self.position_0.x = Tre[0]
		self.position_0.y = Tre[1]
		self.position_0.z = 1 - Tre[2]
		self.position_0.yaw = theta_y

		self.kalman.kalman_update(self.position_0)
	    if data.tag[i].id == 1 :
		Rt = np.asarray(data.tag[i].pose.r.element)  
		T = np.asarray(data.tag[i].pose.t.element)
		T.shape=(1,3)
		T = T.T
		Rt = Rt.reshape((3,3),order='C')
		Rt_inv = np.linalg.inv(Rt)
		print "!=================================="
		print Rt_inv
		print T
		print np.matmul(Rt_inv, T)#np.multiply(Rt_inv, T)
		Tre = np.matmul(Rt_inv, T)

		r21 = data.tag[i].pose.r.element[2 * 3 + 1]
  		r22 = data.tag[i].pose.r.element[2 * 3 + 2]
  		sy = math.sqrt(r21 * r21 + r22 * r22)
  		theta_y = math.atan2(-data.tag[i].pose.r.element[2 * 3 + 0], sy)

		self.position_1.x =  0.29 + Tre[0]
		self.position_1.y = Tre[1]
		self.position_1.z = 1 - Tre[2]
		self.position_1.yaw = theta_y
	    if data.tag[i].id == 2 :
		Rt = np.asarray(data.tag[i].pose.r.element)  
		T = np.asarray(data.tag[i].pose.t.element)
		T.shape=(1,3)
		T = T.T
		Rt = Rt.reshape((3,3),order='C')
		Rt_inv = np.linalg.inv(Rt)
		print "!=================================="
		print Rt_inv
		print T
		print np.matmul(Rt_inv, T)#np.multiply(Rt_inv, T)
		Tre = np.matmul(Rt_inv, T)

		r21 = data.tag[i].pose.r.element[2 * 3 + 1]
  		r22 = data.tag[i].pose.r.element[2 * 3 + 2]
  		sy = math.sqrt(r21 * r21 + r22 * r22)
  		theta_y = math.atan2(-data.tag[i].pose.r.element[2 * 3 + 0], sy)

		self.obstacle_pos.x = -Tre[0]
		self.obstacle_pos.y = Tre[1]
		self.obstacle_pos.z = -Tre[2]
		self.obstacle_pos.yaw = theta_y
		self.obstacle_flag = 1
	if (len(data.tag)) == 0:
	    self.pos.x = -1
	    self.pos.y = -1
	    self.pos.z = -1
	elif (len(data.tag)) == 2:
	    self.pos.x = (0 * self.position_1.x + 2 * self.position_0.x) / 2
	    self.pos.y = (0 * self.position_1.y + 2 * self.position_0.y) / 2
	    self.pos.z = (0 * self.position_1.z + 2 * self.position_0.z) / 2
	elif (len(data.tag)) == 1:
	    if (data.tag[0].id == 0):
		self.pos.x = self.position_0.x
		self.pos.y = self.position_0.y
		self.pos.z = self.position_0.z
	    if (data.tag[0].id == 1):
		#self.pos.x = self.position_1.x
		#self.pos.y = self.position_1.y
		#self.pos.z = self.position_1.z
		pass
	self.localization.apriltag0.x = self.position_0.x
	self.localization.apriltag0.y = self.position_0.y
	self.localization.apriltag0.z = self.position_0.z
	self.localization.apriltag0.yaw = self.position_0.yaw
	self.localization.apriltag1.x = self.position_1.x
	self.localization.apriltag1.y = self.position_1.y
	self.localization.apriltag1.z = self.position_1.z
	self.localization.apriltag1.yaw = self.position_1.yaw
	self.localization.apriltag.x = self.pos.x
	self.localization.apriltag.y = self.pos.y
	self.localization.apriltag.z = self.pos.z
	self.localization.apriltag.yaw = self.pos.yaw
	self.kalman.kalman_update(self.localization.apriltag0);

    def localization_with_odometer_calculation(self):
        # this needs to be done by student
        self.localization.predict.z += self.speed * 0.05 * math.cos(self.yaw)
        self.localization.predict.y = 0
        self.localization.predict.x += self.speed * 0.05 * math.sin(self.yaw)
        self.localization.predict.yaw = self.yaw


if __name__ == '__main__':
    cyber.init()
    exercise_node = cyber.Node("localization_node")
    exercise = Exercise(exercise_node)
    exercise_node.spin()
    cyber.shutdown()

