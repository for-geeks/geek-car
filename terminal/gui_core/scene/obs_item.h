#ifndef BAIDU_ACU_CPTCONTROL_OBSTACLE_OBS_ITEM_H
#define BAIDU_ACU_CPTCONTROL_OBSTACLE_OBS_ITEM_H
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QTimer>
//QGraphicsScene管理QGraphicsItem（单击/选择/移动/缩放/删除）
// 自定义 Item
class CustomItem : public QGraphicsRectItem
{
public:
    explicit CustomItem(QGraphicsItem *parent = 0);
    int type() const;
protected:
    // Shift+左键：进行选择  Alt：准备缩放
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    // Alt+拖拽：进行缩放  移动
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    // 使item可使用qgraphicsitem_cast
private:
    QPointF m_centerPointF;
    bool m_bResizing;
};
// 自定义 Scene
class CustomScene : public QGraphicsScene
{
    Q_OBJECT
private slots:
    void timer_update(void);
    void update_brake(float brake){
        stop_acc = 2 * brake / 100;
    }
signals:
    void brake_update_notify(float);
    void throttle_update_notify(float);
    void steer_update_notify(float);
private:
    QTimer * timer;

    void update_key_value(void);
public:
    std::vector<std::pair<float, float>> get_item(void){
        QList<QGraphicsItem*> list = this->items();
        std::vector<std::pair<float, float>> item_vec;
        item_vec.clear();
        for (int i = 0; i < list.size(); i++){
            CustomItem test;
            if (list.at(i)->type() == test.type()){
                item_vec.push_back(std::make_pair(list.at(i)->pos().x(), list.at(i)->pos().y()));
            }
        }
        return item_vec;
    }

    QGraphicsPixmapItem *s_steer_pic_item;
    CustomScene(bool flag = true){
        mesh_init();
        timer = new QTimer(this);
        this->connect(timer, SIGNAL(timeout()), this, SLOT(timer_update()));
        timer->start(25);
        if (flag){
            const char* CAR_PIC = "./pic/car.jpeg";
            QPixmap * qpixm = new QPixmap(CAR_PIC);
            *qpixm = qpixm->scaled(QSize(46, 90), Qt::IgnoreAspectRatio);
            s_steer_pic_item = new QGraphicsPixmapItem(*qpixm);
            s_steer_pic_item->setTransformOriginPoint(22,
                                                      70);
            s_steer_pic_item->setScale(1);

            this->addItem(s_steer_pic_item);
        }
    }
    ~CustomScene(void){}
    bool s_w_pressflag = false;
    bool s_s_pressflag = false;
    bool s_a_pressflag = false;
    bool s_d_pressflag = false;
    bool s_x_pressflag = false;
    float steer_cmd = 0;
    float brake_cmd = 0;
    float throttle_cmd = 0;
    float speed_cmd_virtual = 0;
    float stop_acc = 0;
    uint8_t direction = 0;
    void display_car(float x, float y, float yaw);
    void mesh_init(void){
        for (int u = 0; u < this->height(); u++)
        {
            if (u % 100 == 0)
            {
                if (u == 0){
                    this->addLine(10, this->height() - (u + 10),
                        this->width(), this->height() - (u + 10), QPen(QColor(255, 0, 0)));
                }else{
                    this->addLine(10, this->height() - (u + 10),
                        this->width(), this->height() - (u + 10), QPen(QColor(0, 0, 255)));
                }
                QGraphicsTextItem * text_item = this->addText(QString::number(u).sprintf("%d", u));
                text_item->setPos(0 - 20, this->height() - (u + 30));
            }
        }
        //绘制竖直方向的直线
        for (int t = 0; t < this->width(); t++)
        {
            if (t % 100 == 0)
            {
                if (t == 0){
                    this->addLine(t + 10, this->height() - 10, t + 10,
                        this->height() - this->height() + 10, QPen(QColor(255, 0, 0)));
                }else{
                    this->addLine(t + 10, this->height() - 10, t + 10,
                        this->height() - this->height() + 10, QPen(QColor(0, 0, 255)));
                }
            }
        }
    }
    std::vector<QGraphicsLineItem *> line_items;
    void display_points(std::vector<std::pair<float, float>> points);
    void display_sonar_obs(std::vector<std::pair<float, float>> points);
    void display_stop_point(float x, float y){
        static QGraphicsRectItem * rect_items = nullptr;
        if (rect_items != nullptr){
            delete rect_items;
            rect_items = nullptr;
        }
        rect_items = this->addRect(x - 5, y - 5,
                      10, 10, QColor(255, 0, 0));
    }
    void display_freespace_obs(std::vector<std::pair<float, float>> points);
    void display_visual_obs(std::vector<std::pair<float, float>> blpoints,
                            std::vector<std::pair<float, float>> tlpoints,
                            std::vector<std::pair<float, float>> trpoints,
                            std::vector<std::pair<float, float>> brpoints);
    void display_roi(std::vector<std::pair<float, float>> points);
    void display_visual_points(std::vector<std::pair<float, float>> points, bool clear_flag);
protected:
    // 左键：添加item  右键：移除item
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    // Backspace键移除item
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *ev);

};
#endif
