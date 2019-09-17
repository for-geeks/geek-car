#include "lateral_control_exercise.h"
#include "ui_lateral_control_exercise.h"
#include <iostream>
#include <QGraphicsScene>
int* lateral_control_exercise_ptr = nullptr;
//lateral_control_exercise *ptr = nullptr;
#include "obs_item.h"
#include <QScrollBar>
#include <QGraphicsItem>

static int obs_cb(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (lateral_control_exercise_ptr != nullptr){
        ((lateral_control_exercise *)lateral_control_exercise_ptr)->update_obs((char*)data->buff, data->len);
    }
    return 0;
}

static int pose_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (lateral_control_exercise_ptr != nullptr){
        ((lateral_control_exercise *)lateral_control_exercise_ptr)->update_pose((unsigned char*)data->buff, data->len);
    }
    return 0;
}

static int trajectory_feedback_cb(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (lateral_control_exercise_ptr != nullptr){
        ((lateral_control_exercise *)lateral_control_exercise_ptr)->update_trajectory((char*)data->buff, data->len);
    }
    return 0;
}

static int chassie_feedback_cb(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (lateral_control_exercise_ptr != nullptr){
        ((lateral_control_exercise *)lateral_control_exercise_ptr)->update_chassie_info((char*)data->buff, data->len);
    }
    return 0;
}

lateral_control_exercise::lateral_control_exercise(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::lateral_control_exercise)
{
    ui->setupUi(this);
    qtimer = new QTimer();
    qtimer->setInterval(50);
    qtimer->setTimerType(Qt::PreciseTimer);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(_timer_update()));
    qtimer->start();
    control = new ControlButton(ui->graphicsView_control);
    std::map<umb_cb_recv, std::pair<int, int>> sub_list;
    std::map<std::string, std::pair<int, int>> pub_list;
    sub_list.clear();
    pub_list.clear();
    sub_list.insert(std::make_pair(pose_cb_recv, std::make_pair(1035, 2)));
    pub_list.insert(std::make_pair("control_cmd", std::make_pair(1035, 3)));
    pub_list.insert(std::make_pair("target_obs", std::make_pair(1035, 9)));
    sub_list.insert(std::make_pair(chassie_feedback_cb, std::make_pair(1035, 4)));
    sub_list.insert(std::make_pair(trajectory_feedback_cb, std::make_pair(1035, 10)));
    sub_list.insert(std::make_pair(obs_cb, std::make_pair(1035, 12)));
    //pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
    umb_center = new UmbStandard(sub_list, pub_list);
    scene = new CustomScene();
    scene_lane = new CustomScene();
    scene_lane->s_steer_pic_item->~QGraphicsItem();
    scene->setSceneRect(0, 0, ui->graphicsView_pos->width() * 5, ui->graphicsView_pos->height() * 5);
    ui->graphicsView_pos->setScene(scene);
    scene_lane->setSceneRect(0, 0, ui->graphicsView_lane->width(), ui->graphicsView_lane->height());
    ui->graphicsView_lane->setScene(scene_lane);
    scene_lane->mesh_init();
    scene->mesh_init();
    memset(trajectory_pts, 0, 50 * sizeof(Point2d_s));
    //scene_lane->s_steer_pic_item->setPos(-22 + scene_lane->sceneRect().center().x(), -70 + scene_lane->sceneRect().center().y());
}

void lateral_control_exercise::update_pose(unsigned char * buff, unsigned int length){

    if (sizeof(pose_reader) < length){
        return;
    }
    memcpy(&pose_reader, buff, length);
}

lateral_control_exercise::~lateral_control_exercise()
{
    delete ui;
    delete umb_center;
}
void lateral_control_exercise::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(8);
    std::cout << "closing "<< std::endl;
}
void lateral_control_exercise::_display_position(void){

}
void lateral_control_exercise::_log_trajectory(void){
    static trajectory_t last_trajectory;
    trajectory_t trajectory;

    double q0, q1, q2, q3;
    q1 = pose_reader.rotation.x;
    q2 = pose_reader.rotation.y;
    q3 = pose_reader.rotation.z;
    q0 = pose_reader.rotation.w;
    double yaw = atan((q1 * q2 - q0 * q3) * 2 / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
    trajectory.yaw = point.yaw;
    float x = -pose_reader.translation.z - start_point.x;
    float y = pose_reader.translation.x - start_point.y;
    float L = 0.315;
    trajectory.x = point.x;
    trajectory.y = point.y;
    _display_position();
    if (ui->checkBox_record->isChecked()){
        if (_distance_2d(trajectory, last_trajectory) >= 0.05){
            memcpy(&last_trajectory, &trajectory, sizeof(trajectory));
            traj.push_back(trajectory);
            traj_vec.push_back(std::make_pair(300 * trajectory.x + scene->sceneRect().center().x() ,
                                              300 * trajectory.y + scene->sceneRect().center().y()));
        }
    }
}
void lateral_control_exercise::_timer_update(void){

    float L = 0.315;
    double q0, q1, q2, q3;
    q1 = pose_reader.rotation.x;
    q2 = pose_reader.rotation.y;
    q3 = pose_reader.rotation.z;
    q0 = pose_reader.rotation.w;
    double yaw = atan2((q1 * q2 - q0 * q3) * 2, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    //trajectory.x =
    point.yaw = yaw - start_point.yaw;
    //point.x = -pose_reader.translation.z - start_point.x;
    point.x += chassie_info.speed * 0.05 * cos(point.yaw);
    //point.y = pose_reader.translation.x - start_point.y;
    point.y += chassie_info.speed * 0.05 * sin(point.yaw);
    ui->graphicsView_pos->horizontalScrollBar()->setValue(300 * point.x +
                                                          scene->sceneRect().center().x() -
                                                          ui->graphicsView_pos->width() / 2);
    ui->graphicsView_pos->verticalScrollBar()->setValue(300 *
                                                        point.y + scene->sceneRect().center().y()
                                                        - ui->graphicsView_pos->height() / 2);
    _log_trajectory();
    scene->display_car(point.x, point.y, M_PI / 2 + point.yaw);
    scene->display_points(traj_vec);
    //scene->display_stop_point();
    laneinfo_t lane;
    _calculate_lane(&lane);



}

void lateral_control_exercise::on_checkBox_record_toggled(bool checked)
{
    if (checked){
        traj.clear();
        traj_vec.clear();
        start_point.x = 0;
        start_point.y = 0;
        point.x = 0;
        point.y = 0;
        double q0, q1, q2, q3;
        q1 = pose_reader.rotation.x;
        q2 = pose_reader.rotation.y;
        q3 = pose_reader.rotation.z;
        q0 = pose_reader.rotation.w;
        double yaw = atan2((q1 * q2 - q0 * q3) * 2, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        start_point.yaw = yaw;
    }
}

double lateral_control_exercise::_distance_2d(trajectory_t t1, trajectory_t t2){
    return sqrt((t1.x - t2.x) * (t1.x - t2.x) + (t1.y - t2.y) * (t1.y - t2.y));
}

unsigned int lateral_control_exercise::_find_nearest_idx(float x, float y){
    //static int last_idx = -1;
    //static int this_idx = -1;
    float center_x = 0.0;
    float center_y = 0.0;
    unsigned int idx = 0;
    center_x = x;
    center_y = y;
    float min_dist = 9999.0;
    if (this_idx < 0){
        for (int i = 0; i < traj_vec.size(); i++){
            float dist = (traj.at(i).x - center_x) * (traj.at(i).x - center_x) +
                    (traj.at(i).y - center_y) * (traj.at(i).y - center_y);
            if (i < traj_vec.size() / 5){
                min_dist = dist;
                this_idx = i;
            }
        }
    }else{
        for (int i = 0; i < traj_vec.size(); i++){
            float dist = (traj.at(i).x - center_x) * (traj.at(i).x - center_x) +
                    (traj.at(i).y - center_y) * (traj.at(i).y - center_y);
            if (dist < min_dist){
                if (abs(i - last_idx) < 100){
                    min_dist = dist;
                    this_idx = i;
                }
            }
        }
    }
    last_idx = this_idx;
    return this_idx;
}

std::vector<std::pair<float, float>> lateral_control_exercise::_sense_points(void){
    std::vector<std::pair<float, float>> point_sense;
    std::vector<std::pair<float, float>>::iterator it;
    point_sense.clear();
    unsigned int min_index = _find_nearest_idx(point.x, point.y);
    if (((traj_vec.size() > (min_index + 20))) && (min_index != 9999)){
        for (it = traj_vec.begin() + min_index; it != traj_vec.begin() + min_index + 20; ){
            point_sense.push_back(*it);
            float x = 0;
            float y = 0;
            it++;
        }
    }
    return point_sense;
}

void lateral_control_exercise::_calculate_lane(laneinfo_t *lane){
    std::vector<std::pair<float, float>> point_sense;
    std::vector<std::pair<float, float>> out_point;
    point_sense = _sense_points();
    scene->display_sonar_obs(point_sense);
    out_point = _global2vehicle_asvec(point_sense);
    point_sense = _global2vehicle_asvec(point_sense);
    fitter.least_square(point_sense);
    point_sense = fitter.point_generator();
    for (int i = 0; i < point_sense.size(); i++){
        point_sense.at(i).first = point_sense.at(i).first * 100 +
                scene_lane->sceneRect().center().x() - 0;
                //i->graphicsView_lane->width() / 2;
        point_sense.at(i).second = point_sense.at(i).second * 100 +
                scene_lane->sceneRect().center().y() - 0;
                //ui->graphicsView_lane->height() / 2;
    }
    scene_lane->display_points(point_sense);
    char buff[8];
    float steer = 0;
    //steer = 100 * 10 * (0 + fitter.v[3]);
    printf("atan(fitter.v[2]): %f ============", atan(fitter.v[2]) * 180 / M_PI);

    //steer = 4 * atan(fitter.v[2]) * 180 / M_PI + 100 * 4 * (0 + fitter.v[3]) + 100 * fitter.v[1];
    //steer = 100 * fitter.v[1];
    //4 * atan(fitter.v[2]) * 180 / M_PI + 100 * 4 * (0 + fitter.v[3]) +
    target.start_point.x = 0;
    target.start_point.y = 0;
    std::vector<std::pair<float, float>> traj_vec_planning, plan;
    traj_vec_planning.clear();
    for (int i = 0; i < 50; i++){
        if (fabs(trajectory_pts[i].y) > 0.01){
            traj_vec_planning.push_back(std::make_pair(trajectory_pts[i].y, -trajectory_pts[i].x));
        }
    }
    plan = _vehicle2global_asvec(traj_vec_planning, false);
    scene->display_freespace_obs(plan);

    if (out_point.size()){
        target.end_point.x = -out_point.at(out_point.size() - 1).second;
        target.end_point.y = out_point.at(out_point.size() - 1).first;

        //
    }else{
        target.end_point.x = 0;
        target.end_point.y = 0;
    }
    //scene->
    //umb_center->send_umb_messeage(umb_center->pub_map["control_cmd"], buff, 8);
    QList<QGraphicsItem*> list = scene->items();
    target.obs_points.x = -1;
    target.obs_points.y = -1;
    float x, y;
    for (int i = 0; i < list.size(); i++){
        CustomItem test;
        if (list.at(i)->type() == test.type()){
            _global2vehicle(list.at(i)->pos().x(), list.at(i)->pos().y(),
                            &x, &y);
            target.obs_points.x = -y;
            target.obs_points.y = x;
        }
    }

        //1 * atan(fitter.v[2]) * 180 / M_PI + 100 * 1 * (0 + fitter.v[3]) +
        //float x_tar =
        //std::cout << target.obs_points.y << " " << (target.obs_points.y > 0.0 <<std::endl;
    if (target.obs_points.y > 0.0){
        if (traj_vec_planning.size()){
            steer = 57 * atan2(2 * traj_vec_planning.at(traj_vec_planning.size() /2).second * 0.313,
                               (traj_vec_planning.at(traj_vec_planning.size() /2).first * traj_vec_planning.at(traj_vec_planning.size() /2).first
                                      + traj_vec_planning.at(traj_vec_planning.size() /2).second * traj_vec_planning.at(traj_vec_planning.size() /2).second));
        }
    }else{
        if (out_point.size()){
            steer = 1 * atan(fitter.v[2]) * 180 / M_PI + 100 * 1 * (0 + fitter.v[3]) + -5 + 57 * atan2(2 * out_point.at(out_point.size() -10).second * 0.313,
                               (out_point.at(out_point.size() -10).first * out_point.at(out_point.size() -10).first
                                      + out_point.at(out_point.size() -10).second * out_point.at(out_point.size() -10).second));
        }

    }
    if ((steer) >= 60){
        steer = 60;
    }else if(steer <= -60){
        steer = -60;
    }
    //steer = (steer1 + steer2) / 2;
    std::cout << "steer control :: " << steer << std::endl;
    memcpy(buff, &steer, 4);
    //memcpy(buff, &control->steer_cmd, 4);


    memcpy(buff + 4, &control->throttle_cmd, 4);
    umb_center->send_umb_messeage(umb_center->pub_map["control_cmd"], buff, 8);
    if ((fabs(target.end_point.y) > 0.01) && (fabs(target.end_point.y) < 1)){
        umb_center->send_umb_messeage(umb_center->pub_map["target_obs"], &target, sizeof(target));
    }

}

std::vector<std::pair<float, float>> lateral_control_exercise::_global2vehicle_asvec(
        std::vector<std::pair<float, float>> invec){
    std::vector<std::pair<float, float>> ret;
    ret.clear();
    for (int i = 0; i < invec.size(); i++){
        float x = 0;
        float y = 0;
        _global2vehicle(invec.at(i).first, invec.at(i).second, &x, &y);
        ret.push_back(std::make_pair(x, y));
    }
    return ret;
}
void lateral_control_exercise::_global2vehicle(float x, float y, float * x_trans, float * y_trans){
    float y_temp = (22 - x +
            scene->s_steer_pic_item->pos().x()) / 300.0;
    float x_temp = (70 - y +
            scene->s_steer_pic_item->pos().y()) / 300.0;
    float tempy =
            x_temp *
            sin(scene->s_steer_pic_item->rotation() * M_PI / 180) +
            y_temp *
            cos(scene->s_steer_pic_item->rotation() * M_PI / 180);
    float tempx =
            x_temp *
            cos(scene->s_steer_pic_item->rotation() * M_PI / 180) -
            y_temp *
            sin(scene->s_steer_pic_item->rotation() * M_PI / 180);
    *y_trans = tempy;
    *x_trans = tempx;
}

std::vector<std::pair<float, float>> lateral_control_exercise::_vehicle2global_asvec(
        std::vector<std::pair<float, float>> invec, bool mode){
    std::vector<std::pair<float, float>> ret;
    ret.clear();
    for (int i = 0; i < invec.size(); i++){
        float x = 0;
        float y = 0;
        _vehicle2global(invec.at(i).first, invec.at(i).second, &x, &y, mode);
        ret.push_back(std::make_pair(x, y));
    }
    return ret;
}

void lateral_control_exercise::_vehicle2global(float x, float y, float * x_trans, float * y_trans, bool mode){
    float centerx_trans = x * cos(scene->s_steer_pic_item->rotation() * -M_PI / 180) -
            y * sin(scene->s_steer_pic_item->rotation() * -M_PI / 180);
    float centery_trans = x * sin(scene->s_steer_pic_item->rotation() * -M_PI / 180) +
            y * cos(scene->s_steer_pic_item->rotation() * -M_PI / 180);
    *x_trans = 22 + scene->s_steer_pic_item->pos().x() +
            (-centery_trans * 300);
    *y_trans = 70 + scene->s_steer_pic_item->pos().y() +
            (-centerx_trans * 300);
}

void lateral_control_exercise::on_pushButton_2_clicked()
{
    trajectory_t trajectory;
    trajectory.x = point.x;
    trajectory.y = point.y;
    trajectory.yaw = 0;
    float distance = 1;
    float step = 0.035;
    float radius = M_PI / 4;
    float step_sum = 0;
    float yaw = M_PI / 2;
    traj.clear();
    traj_vec.clear();
    if (number == 8){
        for (int j = 0; j < 2; j++){
            for (int i = 0; i < distance / step; i++){
                step_sum += step;
                trajectory.x += step * cos(trajectory.yaw);
                trajectory.y += step * sin(trajectory.yaw);
                //trajectory.yaw = M_PI / 2;
                traj.push_back(trajectory);
                traj_vec.push_back(std::make_pair(300 * trajectory.x + scene->sceneRect().center().x() ,
                                                  300 * trajectory.y + scene->sceneRect().center().y()));
            }
            for (int i = 0; i < radius * M_PI / step; i++){

                trajectory.x += step * cos(trajectory.yaw);
                trajectory.y += step * sin(trajectory.yaw);
                trajectory.yaw -= M_PI / (radius * M_PI / step);
                traj.push_back(trajectory);
                traj_vec.push_back(std::make_pair(300 * trajectory.x + scene->sceneRect().center().x() ,
                                                  300 * trajectory.y + scene->sceneRect().center().y()));
            }
        }
    }else{
        for (int i = 0; i < 5 / step; i++){
            step_sum += step;
            trajectory.x += step * cos(trajectory.yaw);
            trajectory.y += step * sin(trajectory.yaw);
            //trajectory.yaw = M_PI / 2;
            traj.push_back(trajectory);
            traj_vec.push_back(std::make_pair(300 * trajectory.x + scene->sceneRect().center().x() ,
                                              300 * trajectory.y + scene->sceneRect().center().y()));
        }
    }
}
