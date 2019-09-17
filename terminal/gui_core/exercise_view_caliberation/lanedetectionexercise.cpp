#include "lanedetectionexercise.h"
#include "ui_lanedetectionexercise.h"
#include <iostream>

int* LaneDetectionExercise_ptr = nullptr;

static int lane_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (LaneDetectionExercise_ptr != nullptr){
        ((LaneDetectionExercise *)LaneDetectionExercise_ptr)->update_lane((char*)data->buff, data->len);
    }
    return 0;
}

LaneDetectionExercise::LaneDetectionExercise(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LaneDetectionExercise)
{
    ui->setupUi(this);
    ui->setupUi(this);
    qtimer = new QTimer();
    qtimer->setInterval(50);
    qtimer->setTimerType(Qt::PreciseTimer);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(_time_update()));
    qtimer->start();
    std::map<umb_cb_recv, std::pair<int, int>> sub_list;
    std::map<std::string, std::pair<int, int>> pub_list;
    sub_list.clear();
    pub_list.clear();
    sub_list.insert(std::make_pair(lane_cb_recv, std::make_pair(1035, 7)));
    umb_center = new UmbStandard(sub_list, pub_list);
    scene = new CustomScene(false);
    scene->setSceneRect(0, 0, ui->graphicsView->width(), ui->graphicsView->height());
    ui->graphicsView->setScene(scene);
    listview = new ListViewStandard(ui->treeWidget);
}

void LaneDetectionExercise::_time_update(void){
    listview->display_item("lane_left", "a", laneinfo.left_lane.a);
    listview->display_item("lane_left", "b", laneinfo.left_lane.b);
    listview->display_item("lane_left", "c", laneinfo.left_lane.c);
    listview->display_item("right_lane", "a", laneinfo.right_lane.a);
    listview->display_item("right_lane", "b", laneinfo.right_lane.b);
    listview->display_item("right_lane", "c", laneinfo.right_lane.c);
    listview->display_item("right_lane_view", "a", laneinfo.right_lane_view.a);
    listview->display_item("right_lane_view", "b", laneinfo.right_lane_view.b);
    listview->display_item("right_lane_view", "c", laneinfo.right_lane_view.c);
    listview->display_item("left_lane_view", "a", laneinfo.left_lane_view.a);
    listview->display_item("left_lane_view", "b", laneinfo.left_lane_view.b);
    listview->display_item("left_lane_view", "c", laneinfo.left_lane_view.c);
    std::vector<std::pair<float, float>> left_points;
    std::vector<std::pair<float, float>> right_points;
    left_points.clear();
    right_points.clear();
    for (int i = 0; i < 420; i++){
        left_points.push_back(std::make_pair((float)i,
                                             laneinfo.left_lane_view.a * i * i +
                                             laneinfo.left_lane_view.b * i +
                                             laneinfo.left_lane_view.c));
        right_points.push_back(std::make_pair((float)i,
                                             laneinfo.right_lane_view.a * i * i +
                                             laneinfo.right_lane_view.b * i +
                                             laneinfo.right_lane_view.c));
    }
    scene->display_points(left_points);
    scene->display_freespace_obs(right_points);

}

LaneDetectionExercise::~LaneDetectionExercise()
{
    delete ui;
    delete umb_center;
}
void LaneDetectionExercise::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(3);
    std::cout << "closing "<< std::endl;
}
