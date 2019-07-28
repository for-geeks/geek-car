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

class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Tags()
	self.localization = localization()
	self.position_0 = pos()
	self.position_1 = pos()
	self.pos = pos()
	self.marker_pos = {0: [1.0, 0], 1: [2.0, 0]}
        self.node.create_reader("/localization/tag", Tags, self.callback) 
        self.node.create_reader("/realsense/pose", Pose, self.posecallback) 
        self.node.create_reader("/chassis", Chassis, self.Chassiscallback) 
        self.writer = self.node.create_writer("/localization", localization)
	self.start_yaw = 0
	self.init_flag = 0;
	self.speed = 0
	self.yaw = 0
	while not cyber.is_shutdown():
	    time.sleep(0.05)
	    self.localization_with_odometer_calculation();
	    self.writer.write(self.localization)
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
            self.start_yaw = math.atan2((q1 * q2 - q0 * q3) * 2, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
	    self.init_flag = 1
	    self.localization.predict.z = 0
	    self.localization.predict.y = 0
	    self.localization.predict.x = 0
	    self.localization.predict.yaw = 0
	    #start_yaw
	
    def localization_with_odometer_calculation(self):
	#this needs to be done by student
	self.localization.predict.z += self.speed * 0.05 * math.cos(self.yaw)
	self.localization.predict.y = 0
	self.localization.predict.x += self.speed * 0.05 * math.sin(self.yaw)
	self.localization.predict.yaw = self.yaw

    def get_global_pos_by_apriltag(self, data): 
        pitch = -15 * 3.14159265358979 / 180
	print math.sin(pitch * 2)
	for i in range(0, len(data.tag)):
	    if (data.tag[i].id == 0):
	        x = data.tag[i].pose.t.element[0];#self.marker_pos[1][1] - 
	        self.position_0.y = math.cos(pitch) * data.tag[i].pose.t.element[1] - math.sin(pitch) * data.tag[i].pose.t.element[2];
	        z = (math.sin(pitch) * data.tag[i].pose.t.element[1] + math.cos(pitch) * data.tag[i].pose.t.element[2]);#self.marker_pos[1][0] - 
		r21 = data.tag[i].pose.r.element[2 * 3 + 1]
  		r22 = data.tag[i].pose.r.element[2 * 3 + 2]
  		sy = math.sqrt(r21 * r21 + r22 * r22)
  		theta_y = math.atan2(-data.tag[i].pose.r.element[2 * 3 + 0], sy)
		self.position_0.x = 0 - math.cos(-theta_y) * x + math.sin(-theta_y) * z;
		self.position_0.z = 1 - math.sin(-theta_y) * x - math.cos(-theta_y) * z;
		self.position_0.yaw = theta_y
	    if data.tag[i].id == 1 :
	        x = self.marker_pos[1][1] - data.tag[i].pose.t.element[0];
	        self.position_1.y = math.cos(pitch) * data.tag[i].pose.t.element[1] - math.sin(pitch) * data.tag[i].pose.t.element[2];
	        z = self.marker_pos[1][0] - (math.sin(pitch) * data.tag[i].pose.t.element[1] + math.cos(pitch) * data.tag[i].pose.t.element[2]);
		r21 = data.tag[i].pose.r.element[2 * 3 + 1]
  		r22 = data.tag[i].pose.r.element[2 * 3 + 2]
  		sy = math.sqrt(r21 * r21 + r22 * r22)
  		theta_y = math.atan2(-data.tag[i].pose.r.element[2 * 3 + 0], sy)
		self.position_1.x = self.marker_pos[1][1] - math.cos(theta_y) * x + math.sin(theta_y) * z;
		self.position_1.z = self.marker_pos[1][0] - math.sin(theta_y) * x - math.cos(theta_y) * z;
		self.position_1.yaw = theta_y
	if (len(data.tag)) == 0:
	    self.pos.x = -1
	    self.pos.y = -1
	    self.pos.z = -1
	elif (len(data.tag)) == 2:
	    self.pos.x = (self.position_1.x + self.position_0.x) / 2
	    self.pos.y = (self.position_1.y + self.position_0.y) / 2
	    self.pos.z = (self.position_1.z + self.position_0.z) / 2
	elif (len(data.tag)) == 1:
	    if (data.tag[0].id == 0):
		self.pos.x = self.position_0.x
		self.pos.y = self.position_0.y
		self.pos.z = self.position_0.z
	    if (data.tag[0].id == 1):
		self.pos.x = self.position_1.x
		self.pos.y = self.position_1.y
		self.pos.z = self.position_1.z
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
	

if __name__ == '__main__':
    cyber.init()

    # TODO update node to your name
    exercise_node = cyber.Node("localization_node")
    exercise = Exercise(exercise_node)
    exercise_node.spin()
    cyber.shutdown()


