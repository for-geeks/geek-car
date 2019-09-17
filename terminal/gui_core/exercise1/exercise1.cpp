#include "exercise1.h"
#include <iostream>
#include <map>
#include <utility>
#include <QtGui/QImage>
#include "share_mem/share_memory.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
#include "list_view_standard.h"
#include <string>
#include "control_button.h"
#include <QLayout>
//#include <stdio.h>
#include<unistd.h>
//#include <GL/glu.h>
//#include <process.h>
// 用于创建一个唯一的key
// 消息结构


void MessageCom::msg_init(void)
{

    // 循环读取消息
    return;
}

int* exercise1_ptr = nullptr;
exercise1 *ptr = nullptr;

static int transformimage_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (exercise1_ptr != nullptr){
        ((exercise1 *)exercise1_ptr)->update_image_trans((unsigned char*)data->buff, data->len);
    }
    return 0;
}

static int image_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (exercise1_ptr != nullptr){
        ((exercise1 *)exercise1_ptr)->msgcom.read_image();
    }
    return 0;
}

static int pose_cb_recv(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (exercise1_ptr != nullptr){
        ((exercise1 *)exercise1_ptr)->update_pose((unsigned char*)data->buff, data->len);
    }
    return 0;
}

void send_image2qt(unsigned char * location, int width, int height, int size){

    if (((exercise1 *)exercise1_ptr) != nullptr){
        std::cout << "get in" << std::endl;
        ((exercise1 *)exercise1_ptr)->update_image(location, width, height, size);
    }else{
        std::cout << "no ready" << std::endl;
    }
}

void exercise1::update_pose(unsigned char * buff, unsigned int length){

    if (sizeof(pose_reader) < length){
        return;
    }
    memcpy(&pose_reader, buff, length);

}

exercise1::exercise1(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exercise1)
{
    ui->setupUi(this);
    qtimer = new QTimer(this);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(_timer_update()));
    qtimer->setInterval(50);
    qtimer->setTimerType(Qt::PreciseTimer);
    qtimer->start();
    exercise1_ptr = (int*)this;
    ptr = this;
    std::cout << "exercise 1 ready" << std::endl;
    //start_listener_cyber();
    //std::cout << (int)ptr;
    std::map<umb_cb_recv, std::pair<int, int>> sub_list;
    std::map<std::string, std::pair<int, int>> pub_list;
    sub_list.clear();
    pub_list.clear();
    sub_list.insert(std::make_pair(image_cb_recv, std::make_pair(1035, 1)));
    sub_list.insert(std::make_pair(transformimage_cb_recv, std::make_pair(1035, 13)));
    sub_list.insert(std::make_pair(pose_cb_recv, std::make_pair(1035, 2)));
    pub_list.insert(std::make_pair("control_cmd", std::make_pair(1035, 3)));
    //pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
    umb_center = new UmbStandard(sub_list, pub_list);
    listview = new ListViewStandard(ui->treeWidget);
    control = new ControlButton(ui->graphicsView_2);
    cube = new VowelCube();
    cube->setMinimumSize(200, 200);
    cube->resize(300, 300);
    ui->gridLayout->addWidget(cube);
}

void exercise1::_timer_update(void){
    //std::cout << "exercise 1 ready" << std::endl;
    _display_altitude();
}

void exercise1::_display_altitude(void){

    listview->display_item("velocity", "x", pose_reader.velocity.x);
    listview->display_item("velocity", "y", pose_reader.velocity.y);
    listview->display_item("velocity", "z", pose_reader.velocity.z);

    listview->display_item("translation", "x", pose_reader.translation.x);
    listview->display_item("translation", "y", pose_reader.translation.y);
    listview->display_item("translation", "z", pose_reader.translation.z);

    listview->display_item("rotation", "x", pose_reader.rotation.x);
    listview->display_item("rotation", "y", pose_reader.rotation.y);
    listview->display_item("rotation", "z", pose_reader.rotation.z);
    listview->display_item("rotation", "w", pose_reader.rotation.w);
    double q0, q1, q2, q3;
    q1 = pose_reader.rotation.x;
    q2 = pose_reader.rotation.y;
    q3 = pose_reader.rotation.z;
    q0 = pose_reader.rotation.w;
    double yaw = atan((q1 * q2 - q0 * q3) * 2 / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
    listview->display_item("yaw", "yaw", yaw);
    listview->display_item("angular_velocity", "x", pose_reader.angular_velocity.x);
    listview->display_item("angular_velocity", "y", pose_reader.angular_velocity.y);
    listview->display_item("angular_velocity", "z", pose_reader.angular_velocity.z);

    listview->display_item("tracker_confidence", "", pose_reader.tracker_confidence);

    cube->set_rotation(pose_reader.rotation.x,
                       pose_reader.rotation.y, pose_reader.rotation.z);
    char buff[8];
    memcpy(buff, &control->steer_cmd, 4);
    memcpy(buff + 4, &control->throttle_cmd, 4);
    umb_center->send_umb_messeage(umb_center->pub_map["control_cmd"], buff, 8);
    //ui->listView->
    //->
}

exercise1::~exercise1()
{
    delete ui;
    delete umb_center;
}

void exercise1::on_exercise1_destroyed()
{
    std::cout << "closing "<< std::endl;
}

void exercise1::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(1);
    std::cout << "closing "<< std::endl;
}

void exercise1::update_image(unsigned char * location, int width, int height, int size){
    static bool init_flag = false;
    static QGraphicsScene *scene = nullptr;//new QGraphicsScene();
    static QPainter *painter = nullptr;//new QPainter(&qImage);
    static QGraphicsPixmapItem *item;

    if (raw_image == nullptr){
        raw_image = new QImage(location, width, height,
             QImage::Format_Grayscale8);
        scene = new QGraphicsScene();
        painter = new QPainter(raw_image);
        //std::cout << "raw image is null" << std::endl;
    }else{
        //raw_image->fromData(location, width * height);
        //std::cout << "raw image load from data" << std::endl;
        //std::cout << "width : " << width << "height is : " << height << std::endl;
    }

    if (painter != nullptr){
        pix.loadFromData((const uchar*)location, size);
        //pix = QPixmap::fromImage(*raw_image);
        //pix = pix.scaled(ui->label->width(), ui->label->height());
        if (init_flag == false){
            item = new QGraphicsPixmapItem(pix);

            init_flag = true;
        }
        ui->label->setPixmap(pix);
        std::cout << "painting " << std::endl;
    }else{
        std::cout << "painter image is null" << std::endl;
    }
}

void exercise1::update_image_trans(unsigned char * buff, unsigned int lenth){
    static compress_image_t image;
    memcpy(&image, buff, lenth);
    std::cout << image.length << std::endl;
    if (image.length >= 3000){
        pix_trans.loadFromData((const uchar*)image.data, image.length);
    }

}
