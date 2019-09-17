#include "localization.h"
#include "ui_localization.h"
#include <iostream>
#include "obs_item.h"
#include <QGraphicsScene>
int* Localization_ptr = nullptr;

static int localization_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (Localization_ptr != nullptr){
        ((Localization *)Localization_ptr)->update_localiztion((char*)data->buff, data->len);
    }
    return 0;
}

static int kalman_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (Localization_ptr != nullptr){
        ((Localization *)Localization_ptr)->update_kalman((char*)data->buff, data->len);
    }
    return 0;
}

void Localization::_log_apriltag(void){
    float x = localization.apriltag.x;
    float y = -localization.apriltag.z;
    //traj.push_back(std::make_pair(x, y, 0));
    traj_vec.push_back(std::make_pair(300 * x + scene->sceneRect().center().x() ,
                                      300 * y + scene->sceneRect().center().y()));
    if (traj_vec.size() >= 20){
        std::vector<std::pair<float, float>>::iterator it;
        it = traj_vec.begin();
        traj_vec.erase(it);
    }
}

void Localization::_log_predict(void){
    float x = localization.prediction.x;
    float y = -localization.prediction.z;
    //traj_pre.push_back(point);
    traj_pre_vec.push_back(std::make_pair(300 * x + scene->sceneRect().center().x() ,
                                      300 * y + scene->sceneRect().center().y()));
    if (traj_pre_vec.size() >= 20){
        std::vector<std::pair<float, float>>::iterator it;
        it = traj_pre_vec.begin();
        traj_pre_vec.erase(it);
    }
}

void Localization::_log_kalman(void){
    float x = kalman_pos.x;
    float y = -kalman_pos.z;
    //traj_pre.push_back(point);
    kalman_vec.push_back(std::make_pair(300 * x + scene->sceneRect().center().x() ,
                                      300 * y + scene->sceneRect().center().y()));
    if (kalman_vec.size() >= 20){
        std::vector<std::pair<float, float>>::iterator it;
        it = kalman_vec.begin();
        kalman_vec.erase(it);
    }
}

void Localization::_timer_update(void){
    std::vector<std::pair<float, float>> tag;
    tag.clear();
    tag.push_back(std::make_pair(scene->sceneRect().center().x(), -1 * 300 + scene->sceneRect().center().y()));
    tag.push_back(std::make_pair(0.29 * 300 + scene->sceneRect().center().x(), -1 * 300 + scene->sceneRect().center().y()));
    scene->display_freespace_obs(traj_vec);
    scene->display_sonar_obs(traj_pre_vec);
    scene->display_points(kalman_vec);
    scene->display_roi(tag);
    char buff[8];
    memcpy(buff, &control->steer_cmd, 4);
    memcpy(buff + 4, &control->throttle_cmd, 4);
    umb_center->send_umb_messeage(umb_center->pub_map["control_cmd"], buff, 8);
    static float R1, R2;
    R1 = localization.apriltag0.yaw;
    R2 = localization.apriltag1.yaw;
    static QGraphicsEllipseItem * circle1 = nullptr;
    static QGraphicsEllipseItem* circle2 = nullptr;
    if (circle1 != nullptr){
        delete circle1;
        circle1 = nullptr;
    }
    if (circle2 != nullptr){
        delete circle2;
        circle2 = nullptr;
    }
    //circle1 = scene->addEllipse(scene->sceneRect().center().x() - R1 * 150, scene->sceneRect().center().y() - R1 * 150, R1 * 300 * 2, R1 * 300 * 2);
    //circle2 = scene->addEllipse(scene->sceneRect().center().x() - R2 * 150 - 0.29 * 300,
     //                               scene->sceneRect().center().y()  - R2 * 150, R2 * 300 * 2, R2 * 300 * 2);

}

Localization::Localization(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Localization)
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
    sub_list.insert(std::make_pair(localization_cb_recv, std::make_pair(1035, 8)));
    sub_list.insert(std::make_pair(kalman_cb_recv, std::make_pair(1035, 11)));
    pub_list.insert(std::make_pair("control_cmd", std::make_pair(1035, 3)));
    //sub_list.insert(std::make_pair(chassie_feedback_cb, std::make_pair(1035, 4)));
    //pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
    umb_center = new UmbStandard(sub_list, pub_list);
    scene = new CustomScene();
    //scene_lane = new CustomScene();
    scene->s_steer_pic_item->~QGraphicsItem();
    scene->setSceneRect(0, 0, ui->graphicsView_pos->width() * 5, ui->graphicsView_pos->height() * 5);
    ui->graphicsView_pos->setScene(scene);

    scene->mesh_init();
    //scene->s_steer_pic_item->~QGraphicsItem();
}

Localization::~Localization()
{
    delete ui;
    delete umb_center;
}
void Localization::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(4);
    std::cout << "closing "<< std::endl;
}
