#include "exercise2.h"
#include "ui_exercise2.h"
int* exercise2_ptr = nullptr;

static int chassie_feedback_cb(umb_inst_id_t inst_id_in, umb_msgbuff_t *data, umb_timestamp_t *tm, void *param){
    //std::cout << data->len << std::endl;
    if (exercise2_ptr != nullptr){
        ((exercise2 *)(exercise2_ptr))->update_chassie_info((char*)data->buff, data->len);
    }
    return 0;
}

exercise2::exercise2(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exercise2)
{
    ui->setupUi(this);
    ui->doubleSpinBox->setMaximum(2.0);
    ui->spinBox->setMaximum(20);
    ui->doubleSpinBox->setSingleStep(0.1);
    qtimer = new QTimer();
    connect(qtimer, SIGNAL(timeout()), this, SLOT(timer_update()));
    qtimer->start(50);
    exercise2_ptr = (int*)this;
    std::cout << "exercise 2 ready" << std::endl;
    //start_listener_cyber();
    //std::cout << (int)ptr;
    std::map<umb_cb_recv, std::pair<int, int>> sub_list;
    std::map<std::string, std::pair<int, int>> pub_list;
    sub_list.clear();
    pub_list.clear();
    sub_list.insert(std::make_pair(chassie_feedback_cb, std::make_pair(1035, 4)));
    //sub_list.insert(std::make_pair(pose_cb_recv, std::make_pair(1035, 2)));
    pub_list.insert(std::make_pair("control_cmd", std::make_pair(1035, 3)));
    pub_list.insert(std::make_pair("control_ref", std::make_pair(1035, 5)));
    //pub_list.insert(std::make_pair(std::string("raw_image"), std::make_pair(1035, 1)));
    umb_center = new UmbStandard(sub_list, pub_list);
    listview = new ListViewStandard(ui->treeWidget);
    oscilloscope = new Oscilloscope(ui->graphicsView);
    oscilloscope->init(1);
}

exercise2::~exercise2()
{
    delete ui;
    delete umb_center;
}
void exercise2::closeEvent(QCloseEvent *event)
{
    //TODO: 在退出窗口之前，实现希望做的操作
    //this->~exercise1();
    emit close_sig(7);
    std::cout << "closing "<< std::endl;
}

void exercise2::timer_update(void){
    listview->display_item("chassie", "steer_exec", chassie_info.steer);
    listview->display_item("chassie", "throttle_exec", chassie_info.throttle);
    listview->display_item("chassie", "real_speed", chassie_info.speed);
    oscilloscope->update(chassie_info.speed, ui->doubleSpinBox->value(), "speed", 100, 0);
    char buff[8];
    float thset = ui->spinBox->value();
    float spset = ui->doubleSpinBox->value();
    memset(buff, 0, 4);
    memcpy(buff + 4, &thset, 4);
    if (ui->radioButton->isChecked()){
        umb_center->send_umb_messeage(umb_center->pub_map["control_cmd"], buff, 8);
    }else{
        memset(buff, 0, 4);
        memcpy(buff + 4, &spset, 4);
        umb_center->send_umb_messeage(umb_center->pub_map["control_ref"], buff, 8);
    }
}

void exercise2::on_radioButton_toggled(bool checked)
{

}
