#include "viewcaliberationexercise.h"
#include "ui_viewcaliberationexercise.h"
#include <iostream>
#include "exercise1.h"
#include "umb_standard.h"
#include "lateral_control_exercise.h"
int* ViewCaliberationExercise_ptr = nullptr;

static int get_point_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (ViewCaliberationExercise_ptr != nullptr){
        ((ViewCaliberationExercise *)ViewCaliberationExercise_ptr)->update_trajectory((char*)data->buff, data->len);
    }
    return 0;
}

ViewCaliberationExercise::ViewCaliberationExercise(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ViewCaliberationExercise)
{

    ui->setupUi(this);
    if (exercise01 != nullptr){
        exercise01->~exercise1();
    }

    exercise01 = new exercise1();
    //connect(exercise01, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    //exercise01->show();
    exercise1_ptr = (int*)exercise01;
    qtimer = new QTimer(this);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(_timer_update()));
    qtimer->setInterval(50);
    qtimer->setTimerType(Qt::PreciseTimer);
    qtimer->start();
    scene_origin = new CustomScene(false);
    scene_origin->setSceneRect(0, 0, ui->graphicsView_origin->width(), ui->graphicsView_origin->height());
    ui->graphicsView_origin->setScene(scene_origin);
    pic_item_origin = new QGraphicsPixmapItem();
    pic_item_origin->setPixmap(exercise01->pix);
    scene_origin->addItem(pic_item_origin);
    listview = new ListViewStandard(ui->treeWidget);
    scene_transform = new CustomScene(false);
    scene_transform->setSceneRect(0, 0, ui->graphicsView_perspective->width(), ui->graphicsView_perspective->height());
    ui->graphicsView_perspective->setScene(scene_transform);
    pic_item_trans = new QGraphicsPixmapItem();
    pic_item_trans->setPixmap(exercise01->pix_trans);
    scene_transform->addItem(pic_item_trans);

    std::map<umb_cb_recv, std::pair<int, int>> sub_list;
    std::map<std::string, std::pair<int, int>> pub_list;
    sub_list.clear();
    pub_list.clear();
    sub_list.insert(std::make_pair(get_point_cb_recv, std::make_pair(1035, 14)));
    pub_list.insert(std::make_pair(std::string("get_point"), std::make_pair(1035, 15)));
    //pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
    umb_center = new UmbStandard(sub_list, pub_list);
    memset(trajectory_pts, 0, sizeof(Point2d_s) * 50);
}

ViewCaliberationExercise::~ViewCaliberationExercise()
{
    if (exercise01 != nullptr){
        exercise01->~exercise1();
    }
    delete ui;
}
void ViewCaliberationExercise::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(2);
    std::cout << "closing "<< std::endl;
}
void ViewCaliberationExercise::_timer_update(void){
    //scene_origin->
    pic_item_origin->setPixmap(exercise01->pix);
    pic_item_trans->setPixmap(exercise01->pix_trans);
    std::vector<std::pair<float, float>> origin_point_vec;
    origin_point_vec.clear();
    origin_point_vec = scene_origin->get_item();
    //listview->name_item_map.clear();
    //listview->view->clear();
    memset(trajectory_pts, 0, sizeof(Point2d_s) * 50);
    for (int i = 0; i < origin_point_vec.size(); i++){
        //listview->view->ite
        listview->display_item("points at origin", std::string("x") + std::to_string(i), origin_point_vec.at(i).first);
        listview->display_item("points at origin", std::string("y") + std::to_string(i), origin_point_vec.at(i).second);
    }
    origin_point_vec.clear();
    origin_point_vec = scene_transform->get_item();
    for (int i = 0; i < origin_point_vec.size(); i++){
        listview->display_item("points after transform", std::string("x") + std::to_string(i), origin_point_vec.at(i).first);
        listview->display_item("points after transform", std::string("y") + std::to_string(i), origin_point_vec.at(i).second);
        trajectory_pts[i].x = origin_point_vec.at(i).first;
        trajectory_pts[i].y = origin_point_vec.at(i).second;
    }
    for (int i = 0; i < 50; i++){
        if (fabs(trajectory_pts_in[i].x) <= 0.001){
            if (fabs(trajectory_pts_in[i].y) <= 0.001){
                break;
            }
        }
        listview->display_item("points in car coordinate", std::string("x") + std::to_string(i), trajectory_pts_in[i].x);
        listview->display_item("points in car coordinate", std::string("y") + std::to_string(i), trajectory_pts_in[i].y);
    }
    umb_center->send_umb_messeage(umb_center->pub_map["get_point"], &trajectory_pts, sizeof(Point2d_s) * 50);
}

void ViewCaliberationExercise::on_pushButton_2_clicked()
{
    listview->name_item_map.clear();
    listview->view->clear();
}
