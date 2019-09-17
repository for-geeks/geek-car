/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "listener.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
#include <string>
#include <thread>
#include "umb.h"
#include <map>
#include "gui_core/umb_standard.h"

#include "cyber/cyber.h"
#include "share_mem/share_memory.h"
#include "cyber_bridge/exercise1/proto/sensors.pb.h"
#include "cyber_bridge/exercise1/proto/control.pb.h"
#include "cyber_bridge/exercise1/proto/localization.pb.h"
#include "cyber_bridge/exercise1/proto/planning.pb.h"
//#include "cyber_bridge/exercise1/proto/chassis.pb.h"
//#include "gui_core/exercise1/exercise1.h"
#include <iostream>

u8 buffer[900 * 900 * 2];

typedef struct _position_s{
    double x;
    double y;
    double z;
    double yaw;

}position_s;

typedef struct _localization_s{
    position_s apriltag0;
    position_s apriltag1;
    position_s apriltag;
    position_s prediction;

}localization_s;
// 用于创建一个唯一的key
typedef struct _image_t{
    long type;
    unsigned char data[900 * 900];
    int width;
    int height;
}image_t;

typedef struct _compress_image_t{
    int length;
    int width;
    int height;
    unsigned char data[20000];
    
}compress_image_t;

typedef struct _Point_t{
    double x;
    double y;
    double z;

}Point_t;

typedef struct _Point4_t{
    double x;
    double y;
    double z;
    double w;

}Point4_t;

typedef struct _Coefficient_t{
    double a;
    double b;
    double c;
}Coefficient_t;

typedef struct _Laneinfo_s{
  Coefficient_t left_lane;
  Coefficient_t right_lane;
  Coefficient_t left_lane_view;
  Coefficient_t right_lane_view; 
}Laneinfo_s;

typedef struct _pose_t{
    Point_t velocity;
    Point4_t rotation;
    Point_t angular_velocity;
    Point_t angular_acceleration;
    uint8_t tracker_confidence;
    uint8_t mapper_confidence;
    Point_t translation;
}pose_t;

typedef struct _chassis_t{
    float steer;
    float throttle;
    float speed;
}chassis_t;

typedef struct _Point2d_s {
  // meters
  double x;
  double y;
}Point2d_s;

typedef struct _PlanningInfo_s {
  Point2d_s start_point;
  Point2d_s end_point;
  Point2d_s obs_points;
}PlanningInfo_s;

MessageCom msg_channel;
image_t image_msg;
CShareMemory *csm;
UmbStandard *umb_center;
bool update_flag = false;

void image_sender(void){
    apollo::cyber::Rate rate(10.0);
    while(apollo::cyber::OK()){
        if (update_flag){
            memcpy(buffer, &image_msg, sizeof(image_msg));
            csm->PutToShareMem(buffer, sizeof(image_msg));
            umb_center->send_umb_messeage(umb_center->pub_map["raw_image"], &image_msg, 4);
            //std::cout << "image sent" << std::endl;
            update_flag = false;
        }
        rate.Sleep();
    }
}

bool ref_flag = false;
bool control_flag = false;
bool lane_flag = false;
bool planning_flag = false;
bool get_point_flag = false;
float steer;
float throttle;
float steer_ref;
float speed_ref;
PlanningInfo_s planninginfo;
Point2d_s trajectory_pts[50];

Point2d_s getpoint_pts[50];
static int planning_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    memcpy(&planninginfo, (void *)(data->buff), data->len);
    planning_flag = true;
    return 0;
}

static int laneinfo_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //memcpy(&lane_info, (void *)(data->buff), data->len);
    //lane_flag = true;
    return 0;
}

static int controlrefs_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    memcpy(&speed_ref, (void *)(data->buff + 4), 4);
    ref_flag = true;
    return 0;
}

static int getpoint_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    //double t = 0.0;
    memset(&getpoint_pts, 0, sizeof(getpoint_pts));
    memcpy(&getpoint_pts, data->buff, data->len);

    get_point_flag = true;
    return 0;
    //talker->Write(cmd);
}

static int control_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    //double t = 0.0;
    memcpy(&steer, data->buff, 4);
    memcpy(&throttle, (void *)(data->buff + 4), 4);
    //std::cout << "call back" << steer
    //<< " " << throttle  << std::endl;
    control_flag = true;
    return 0;
    //talker->Write(cmd);
}

void translationCallback(
    const std::shared_ptr<apollo::planning::Trajectory>& msg) {
    int counter = 0;
    Point2d_s translation_ptrs[50];
    std::cout << "traj size " << msg->point_size() << std::endl;
    for (int i = 0; i < msg->point_size(); i++){
	if (i == (50 - 1)){
	    break;
	}
	translation_ptrs[i].x = msg->point(i).x();
	translation_ptrs[i].y = msg->point(i).y();
        counter = i + 1;
    }
    umb_center->send_umb_messeage(umb_center->pub_map["translation_point"], translation_ptrs, counter * sizeof(Point2d_s));
}

void transformCallback(
    const std::shared_ptr<apollo::sensors::Image>& msg) {
    static compress_image_t image_msg_trans;
    image_msg_trans.length = msg->data().length();
    image_msg_trans.width = msg->width();
    image_msg_trans.height = msg->height();
    memcpy(image_msg_trans.data, (unsigned char*)(msg->data().c_str()), msg->data().length());
    std::cout << "commpressed image recieved" << std::endl;
    //std::cout << image_msg.type << std::endl;
    //std::cout << sizeof(image_msg) << std::endl;
    //update_flag = true;
    umb_center->send_umb_messeage(umb_center->pub_map["transformimage"], &image_msg_trans, image_msg_trans.length + 12);
}

void MessageCallback(
    const std::shared_ptr<apollo::sensors::Image>& msg) {
    image_msg.type = msg->data().length();
    image_msg.width = msg->width();
    image_msg.height = msg->height();
    memcpy(image_msg.data, (unsigned char*)(msg->data().c_str()), msg->data().length());
    //std::cout << msg->width() << "  " << msg->height() << std::endl;
    //std::cout << image_msg.type << std::endl;
    std::cout << sizeof(image_msg) << std::endl;
    update_flag = true;
}

void chassisCallback(const std::shared_ptr<apollo::control::Chassis>& msg){
    static chassis_t chassis;
    chassis.steer = msg->steer_angle();
    chassis.throttle = msg->throttle();
    chassis.speed = msg->speed();
    umb_center->send_umb_messeage(umb_center->pub_map["chassis"], &chassis, sizeof(chassis));
}

void kalmanCallback(const std::shared_ptr<apollo::localization::pos>& msg){
    static position_s pos;
    pos.x = msg->x();
    pos.y = msg->y();
    pos.z = msg->z();
    pos.yaw = msg->yaw();
    umb_center->send_umb_messeage(umb_center->pub_map["kalman_pos"], &pos, sizeof(pos));
}

void PoseCallback(const std::shared_ptr<apollo::sensors::Pose>& msg){
    static pose_t pose_now;
    pose_now.velocity.x = msg->velocity().x();
    pose_now.velocity.y = msg->velocity().y();
    pose_now.velocity.z = msg->velocity().z();
    pose_now.rotation.x = msg->rotation().x();
    pose_now.rotation.y = msg->rotation().y();
    pose_now.rotation.z = msg->rotation().z();
    pose_now.rotation.w = msg->rotation().w();
    pose_now.angular_velocity.x = msg->angular_velocity().x();
    pose_now.angular_velocity.y = msg->angular_velocity().y();
    pose_now.angular_velocity.z = msg->angular_velocity().z();
    pose_now.translation.x = msg->translation().x();
    pose_now.translation.y = msg->translation().y();
    pose_now.translation.z = msg->translation().z();
    pose_now.tracker_confidence = msg->tracker_confidence();
    pose_now.mapper_confidence = msg->mapper_confidence();
    umb_center->send_umb_messeage(umb_center->pub_map["pose"], &pose_now, sizeof(pose_now));
    std::cout << msg->rotation().x() << std::endl;
    std::cout << msg->rotation().y() << std::endl;
    std::cout << msg->rotation().z() << std::endl;
    std::cout << msg->rotation().w() << std::endl;
}

void LaneCallback(const std::shared_ptr<apollo::control::LaneInfo>& msg){
    
    static Laneinfo_s lane_info;
    lane_info.left_lane.a = msg->left_lane().a();
    lane_info.left_lane.b = msg->left_lane().b();
    lane_info.left_lane.c = msg->left_lane().c();

    lane_info.right_lane.a = msg->right_lane().a();
    lane_info.right_lane.b = msg->right_lane().b();
    lane_info.right_lane.c = msg->right_lane().c();

    lane_info.left_lane_view.a = msg->left_lane_view().a();
    lane_info.left_lane_view.b = msg->left_lane_view().b();
    lane_info.left_lane_view.c = msg->left_lane_view().c();

    lane_info.right_lane_view.a = msg->right_lane_view().a();
    lane_info.right_lane_view.b = msg->right_lane_view().b();
    lane_info.right_lane_view.c = msg->right_lane_view().c();

    umb_center->send_umb_messeage(umb_center->pub_map["lane"], &lane_info, sizeof(lane_info));
    std::cout << "Lane get" << std::endl;
}


void a_starCallback(const std::shared_ptr<apollo::planning::Trajectory>& msg){
    int counter = 0;
    for (int i = 0; i < msg->point_size(); i++){
	if (i == (50 - 1)){
	    break;
	}
	trajectory_pts[i].x = msg->point(i).x();
	trajectory_pts[i].y = msg->point(i).y();
        counter = i + 1;
    }
    umb_center->send_umb_messeage(umb_center->pub_map["trajectory"], trajectory_pts, counter * sizeof(Point2d_s));
}

void localizationCallback(const std::shared_ptr<apollo::localization::localization>& msg){
    static localization_s localization;
    localization.apriltag0.x = msg->apriltag0().x();
    localization.apriltag0.y = msg->apriltag0().y();
    localization.apriltag0.z = msg->apriltag0().z();
    localization.apriltag0.yaw = msg->apriltag0().yaw();

    localization.apriltag1.x = msg->apriltag1().x();
    localization.apriltag1.y = msg->apriltag1().y();
    localization.apriltag1.z = msg->apriltag1().z();
    localization.apriltag1.yaw = msg->apriltag1().yaw();

    localization.apriltag.x = msg->apriltag().x();
    localization.apriltag.y = msg->apriltag().y();
    localization.apriltag.z = msg->apriltag().z();
    localization.apriltag.yaw = msg->apriltag().yaw();

    localization.prediction.x = msg->predict().x();
    localization.prediction.y = msg->predict().y();
    localization.prediction.z = msg->predict().z();
    localization.prediction.yaw = msg->predict().yaw();

    umb_center->send_umb_messeage(umb_center->pub_map["localization"], &localization, sizeof(localization));
    std::cout << "localization get" << std::endl;
}

int start_cyber(void) {
  // init cyber framework
  //msg_channel.msg_init();
  std::cout << "o yeah !!!!" << std::endl;

  std::map<umb_cb_recv, std::pair<int, int>> sub_list;
  std::map<std::string, std::pair<int, int>> pub_list;
  sub_list.clear();
  pub_list.clear();
  sub_list.insert(std::make_pair(control_cb_recv, std::make_pair(1035, 3)));
  sub_list.insert(std::make_pair(controlrefs_cb_recv, std::make_pair(1035, 5)));
  sub_list.insert(std::make_pair(laneinfo_cb_recv, std::make_pair(1035, 6)));
  sub_list.insert(std::make_pair(planning_cb_recv, std::make_pair(1035, 9)));
  pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
  pub_list.insert(std::make_pair(std::string("pose"), std::make_pair(1035, 2)));
  pub_list.insert(std::make_pair(std::string("chassis"), std::make_pair(1035, 4)));
  pub_list.insert(std::make_pair(std::string("lane"), std::make_pair(1035, 7)));
  pub_list.insert(std::make_pair(std::string("localization"), std::make_pair(1035, 8)));
  pub_list.insert(std::make_pair(std::string("trajectory"), std::make_pair(1035, 10)));
  pub_list.insert(std::make_pair(std::string("kalman_pos"), std::make_pair(1035, 11)));
  pub_list.insert(std::make_pair(std::string("obs_pos"), std::make_pair(1035, 12)));
  pub_list.insert(std::make_pair(std::string("transformimage"), std::make_pair(1035, 13)));
  pub_list.insert(std::make_pair(std::string("translation_point"), std::make_pair(1035, 14)));
  sub_list.insert(std::make_pair(getpoint_cb_recv, std::make_pair(1035, 15)));

  umb_center = new UmbStandard(sub_list, pub_list);
  csm = new CShareMemory("txh", sizeof(image_t) * 2);
  apollo::cyber::Init("exercise1_listener");
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("exercise1_listener");
  auto talker_node = apollo::cyber::CreateNode("control_sender");
  
  // create talker
  auto talker = talker_node->CreateWriter<apollo::control::Control_Command>("/control");
  // create listener
  
  auto listener =
      listener_node->CreateReader<apollo::sensors::Image>(
      "/realsense/compressed_image", MessageCallback);

  auto pose_listener =
      listener_node->CreateReader<apollo::sensors::Pose>(
          "/realsense/pose", PoseCallback);
  auto chassis_listener = 
      listener_node->CreateReader<apollo::control::Chassis>(
          "/chassis", chassisCallback);
  auto lane_listener = 
      listener_node->CreateReader<apollo::control::LaneInfo>(
          "/perception/lane_line", LaneCallback);
  auto localization_listener = 
      listener_node->CreateReader<apollo::localization::localization>(
          "/localization", localizationCallback);

  auto trajectory_listener = 
      listener_node->CreateReader<apollo::planning::Trajectory>(
          "/planning/trajectory", a_starCallback);
  auto kalman_localization_listener = 
      listener_node->CreateReader<apollo::localization::pos>(
          "/localization/kalman_filter", kalmanCallback);

  auto transform_picture_listener = 
      talker_node->CreateReader<apollo::sensors::Image>(
          "/perception/vertical_view", transformCallback);

  auto caliberation_realwordptr_listener = 
      talker_node->CreateReader<apollo::planning::Trajectory>(
          "/perception/translation_point", translationCallback);

  auto control_reference = talker_node->CreateWriter<apollo::control::Control_Reference>("/control_reference");
  auto virtual_lane = talker_node->CreateWriter<apollo::control::Coefficient>("/Coefficient");
  auto target_writer = talker_node->CreateWriter<apollo::planning::PlanningInfo>("/planning/target");
  auto get_point_writer = talker_node->CreateWriter<apollo::planning::Trajectory>("/perception/get_point");
  std::cout << "o yeah !!!!" << std::endl;
  //memset(buffer, 0, sizeof(820*800));
  std::thread t(image_sender);
  //t.join();
  auto cmd = std::make_shared<apollo::control::Control_Command>();
  auto refs = std::make_shared<apollo::control::Control_Reference>();
  auto laneinfo = std::make_shared<apollo::control::Coefficient>();
  
  apollo::cyber::Rate rate(20.0);
  while (apollo::cyber::OK()){
      if (control_flag){
          cmd->set_steer_angle(static_cast<float>(steer));
          cmd->set_throttle(static_cast<float>(throttle));
          talker->Write(cmd);
          std::cout << cmd->steer_angle()
          << " " << cmd->throttle() << std::endl;
          control_flag = false;
      }if (ref_flag){
          refs->set_angular_speed_ref(static_cast<float>(steer_ref)); 
          refs->set_speed_ref(static_cast<float>(speed_ref));
          control_reference->Write(refs);
          ref_flag = false;
      }
      if (lane_flag){
          //laneinfo->set_a(lane_info.a);
          //laneinfo->set_b(lane_info.b);
          //laneinfo->set_c(lane_info.c);
          //virtual_lane->Write(laneinfo);
          lane_flag = false;
          //lane_info
      }
      if (planning_flag){
	  auto planning = std::make_shared<apollo::planning::PlanningInfo>();
          auto start_point = planning->mutable_start_point();
	  start_point->set_x(planninginfo.start_point.x);
	  start_point->set_y(planninginfo.start_point.y);
          auto end_point = planning->mutable_end_point();
	  end_point->set_x(planninginfo.end_point.x);
	  end_point->set_y(planninginfo.end_point.y);
	  
	  auto next_point = planning->add_obs_points();
	  apollo::planning::Point point; 
	  
	  point.set_x(planninginfo.obs_points.x);
	  point.set_y(planninginfo.obs_points.y);
	  next_point->CopyFrom(point);
	  target_writer->Write(planning);
	  std::cout << "target sent" << std::endl;
          planning_flag = false;
      }
      if (get_point_flag){
	  auto send_point = std::make_shared<apollo::planning::Trajectory>();
	  for (int i = 0; i < 50; i++){
	      if (getpoint_pts[i].x == 0){
		  if (getpoint_pts[i].y == 0){
		      break;
	          }
	      }
	      auto next_send_point = send_point->add_point();
	      apollo::planning::Point point;
	      point.set_x(getpoint_pts[i].x);
	      point.set_y(getpoint_pts[i].y);
	      next_send_point->CopyFrom(point);
	  }
	  
	  get_point_writer->Write(send_point);
	  get_point_flag = false;
      }
      //rate.Sleep();
  }
  //memset(image_msg.data, 0, sizeof(image_msg));
  apollo::cyber::WaitForShutdown();

  return 0;
}

void MessageCom::msg_init(void)
{

    // 获取key值
    if ((key = ftok(MSG_FILE, 'z')) < 0)
    {
        perror("ftok error");
        exit(1);
    }
    // 打印key值
    printf("Message Queue - Client key is: %d.\n", key);

    // 打开消息队列
    if ((msqid = msgget(key, IPC_CREAT|0777)) == -1)
    {
        perror("msgget error");
        exit(1);
    }
    printf("My msqid is: %d.\n", msqid);
    printf("My pid is: %d.\n", getpid());
    //return 0;
}

