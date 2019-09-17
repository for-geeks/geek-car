#include <QKeyEvent>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include "obs_item.h"
// 自定义 Item
CustomItem::CustomItem(QGraphicsItem *parent)
    : QGraphicsRectItem(parent)
{
    // 画笔 - 边框色
    QPen p = pen();
    p.setWidth(2);
    p.setColor(QColor(0, 160, 230));
    setPen(p);
    // 画刷 - 背景色
    setBrush(QColor(247, 160, 57));
    // 可选择、可移动
    setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsMovable);
}
void CustomItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (event->modifiers() == Qt::ShiftModifier) {
            qDebug() << "Custom item left clicked with shift key.";
            // 选中 item
            setSelected(true);
        } else if (event->modifiers() == Qt::ControlModifier) {
            qDebug() << "Custom item left clicked with alt key.";
            // 重置 item 大小
            double radius = boundingRect().width() / 2.0;
            QPointF topLeft = boundingRect().topLeft();
            m_centerPointF = QPointF(topLeft.x() + pos().x() + radius, topLeft.y() + pos().y() + radius);
            QPointF pos = event->scenePos();
            qDebug() << boundingRect() << radius << this->pos() << pos << event->pos();
            double dist = sqrt(pow(m_centerPointF.x()-pos.x(), 2) + pow(m_centerPointF.y()-pos.y(), 2));
            if (dist / radius > 0.8) {
                qDebug() << dist << radius << dist / radius;
                m_bResizing = true;
            } else {
                m_bResizing = false;
            }
        } else {
            qDebug() << "Custom item left clicked.";
            QGraphicsItem::mousePressEvent(event);
            event->accept();
        }
    } else if (event->button() == Qt::RightButton) {
        qDebug() << "Custom item right clicked.";
        event->ignore();
    }
}
void CustomItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->modifiers() == Qt::AltModifier) && m_bResizing) {
        QPointF pos = event->scenePos();
        double dist = sqrt(pow(m_centerPointF.x()-pos.x(), 2) + pow(m_centerPointF.y()-pos.y(), 2));
        setRect(m_centerPointF.x()-this->pos().x()-dist, m_centerPointF.y()-this->pos().y()-dist, dist*2, dist*2);
    } else if (event->modifiers() != Qt::AltModifier) {
        qDebug() << "Custom item moved.";
        QGraphicsItem::mouseMoveEvent(event);
        qDebug() << "moved" << pos();
        //this->scenePos()
    }
}
void CustomItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->modifiers() == Qt::AltModifier) && m_bResizing) {
        m_bResizing = false;
    } else {
        QGraphicsItem::mouseReleaseEvent(event);
    }
}
int CustomItem::type() const
{
    return UserType + 1;
}
// 自定义 Scene
void CustomScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug() << "Custom scene clicked.";
    QGraphicsScene::mousePressEvent(event);
    if (!event->isAccepted()) {
        if (event->button() == Qt::LeftButton) {
            // 在 Scene 上添加一个自定义 item
            QPointF point = event->scenePos();
            CustomItem *item = new CustomItem();
            item->setRect(0, 0, 10, 10);
            item->setTransformOriginPoint(-item->rect().center());
            item->setPos(point);
            addItem(item);
        } else if (event->button() == Qt::RightButton) {
            // 检测光标下是否有 item
            QGraphicsItem *itemToRemove = NULL;
            foreach (QGraphicsItem *item, items(event->scenePos())) {
                if (item->type() == QGraphicsItem::UserType+1) {
                    itemToRemove = item;
                    break;
                }
            }
            // 从 Scene 上移除 item
            if (itemToRemove != NULL)
                removeItem(itemToRemove);
        }
    }
}
void CustomScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug() << "Custom scene moved.";
    QGraphicsScene::mouseMoveEvent(event);
}
void CustomScene::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Backspace) {
        // 移除所有选中的 items
        qDebug() << "selected items " << selectedItems().size();
        while (!selectedItems().isEmpty()) {
            removeItem(selectedItems().front());
        }
    } else {
        QGraphicsScene::keyPressEvent(event);
    }
    QKeyEvent *ev = event;
    if (ev->key() == Qt::Key_W)
    {
        if (!ev->isAutoRepeat() && !s_w_pressflag){
            s_w_pressflag = true;
            //throttle_cmd = 0.9 * throttle_cmd + 0.1 * 0.3;
            //std::cout << "w pressed \n " ;
        }
    }
    if (ev->key() == Qt::Key_S)
    {
        if (!ev->isAutoRepeat() && !s_s_pressflag){
            s_s_pressflag = true;
            //brake_cmd = 0.9 * brake_cmd + 0.1 * 0.3;
            //std::cout << "s pressed \n " ;
        }
    }
    if (ev->key() == Qt::Key_A)
    {
        if (!ev->isAutoRepeat() && !s_a_pressflag){
            s_a_pressflag = true;
            //steer_cmd = 0.9 * steer_cmd + 0.1 * 100;
            //std::cout << "a pressed \n " ;
        }
    }
    if (ev->key() == Qt::Key_D)
    {
        if (!ev->isAutoRepeat() && !s_d_pressflag){
            s_d_pressflag = true;
            //steer_cmd = 0.9 * steer_cmd + 0.1 * -100;
            //std::cout << "d pressed \n " ;
        }
    }
    if (ev->key() == Qt::Key_X)
    {
        if (!ev->isAutoRepeat() && !s_x_pressflag){
            s_x_pressflag = true;
            //steer_cmd = 0.1 * steer_cmd + 0.9 * 0.0;
            //std::cout << "d released \n " ;
        }
    }
}
void CustomScene::keyReleaseEvent(QKeyEvent *ev)
{
    if (ev->key() == Qt::Key_W)
    {
        if (!ev->isAutoRepeat() && s_w_pressflag){
            s_w_pressflag = false;
            //throttle_cmd = 0.1 * throttle_cmd + 0.9 * 0;
            //std::cout << "w released \n " ;
        }
    }
    if (ev->key() == Qt::Key_S)
    {
        if (!ev->isAutoRepeat() && s_s_pressflag){
            s_s_pressflag = false;
            //brake_cmd = 0.1 * brake_cmd + 0.9 * 0.0;
            //std::cout << "s released \n " ;
        }
    }
    if (ev->key() == Qt::Key_A)
    {
        if (!ev->isAutoRepeat() && s_a_pressflag){
            s_a_pressflag = false;
            //steer_cmd = 0.1 * steer_cmd + 0.9 * 0.0;
            //std::cout << "a released \n " ;
        }
    }
    if (ev->key() == Qt::Key_D)
    {
        if (!ev->isAutoRepeat() && s_d_pressflag){
            s_d_pressflag = false;
            //steer_cmd = 0.1 * steer_cmd + 0.9 * 0.0;
            //std::cout << "d released \n " ;
        }
    }
    if (ev->key() == Qt::Key_X)
    {
        if (!ev->isAutoRepeat() && s_x_pressflag){
            s_x_pressflag = false;
            //steer_cmd = 0.1 * steer_cmd + 0.9 * 0.0;
            //std::cout << "d released \n " ;
        }
    }
    if (ev->key() == Qt::Key_Up)
    {
        //this->
    }
    return;
}
void CustomScene::update_key_value(void){
    //static float stop_acc = 0;
    if (s_w_pressflag == true){
        direction = 0;
        throttle_cmd = 0.9 * throttle_cmd + 0.1 * 0.3;
        //acc = 0.9 * acc + 0.1 * 0.3;
    }else{
        throttle_cmd = 0.1 * throttle_cmd + 0.1 * 0.0;
    }
    if (s_s_pressflag == true){
        direction = 1;
        brake_cmd = 0.6 * brake_cmd + 0.4 * 0.3;
    }else{
        brake_cmd = 0.1 * brake_cmd + 0.1 * 0.0;
    }
    if (s_x_pressflag == true){
        stop_acc = 0.72;
    }
    if (stop_acc > 0.5){
        stop_acc = 0.72;
    }
    speed_cmd_virtual = 0.9 * speed_cmd_virtual + throttle_cmd - brake_cmd;
    if (speed_cmd_virtual >= 0){
        speed_cmd_virtual -= stop_acc;
        if (speed_cmd_virtual <= 0){
            speed_cmd_virtual = 0;
        }
    }else{
        speed_cmd_virtual += stop_acc;
        if (speed_cmd_virtual >= 0){
            speed_cmd_virtual = 0;
        }
    }
    //speed_cmd_virtual = 0.8 * speed_cmd_virtual ;
    if (s_a_pressflag == true){
        steer_cmd = 0.9 * steer_cmd + 0.1 * 400;
    }else{
        //steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    if (s_d_pressflag == true){
        steer_cmd = 0.9 * steer_cmd + 0.1 * -400;
    }else{
        //steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    if ((s_d_pressflag == false) && (s_a_pressflag == false)){
        steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    //throttle_cmd = 0.9 * throttle_cmd + 0.1 * 0.3;
}
void CustomScene::timer_update(void){
    update_key_value();
    //emit this->brake_update_notify(brake_cmd);
    //emit this->throttle_update_notify(throttle_cmd);
    //emit this->steer_update_notify(steer_cmd);
}
void CustomScene::display_points(std::vector<std::pair<float, float>> points){
    //this->clear();

    for (int i = 0; i < line_items.size(); i++){
        line_items.at(i)->~QGraphicsLineItem();
    }
    line_items.clear();
    if (points.size() > 0){
        for (int i = 0; i < points.size() - 1; i++){
            line_items.push_back(
                this->addLine(points.at(i).first, points.at(i).second,
                              points.at(i + 1).first, points.at(i + 1).second));
        }
    }
}

void CustomScene::display_car(float x, float y, float yaw){
    //this->clear();
    s_steer_pic_item->setPos(-22 + x * 300 + this->sceneRect().center().x(), -70 + y * 300 + this->sceneRect().center().y());
    s_steer_pic_item->setRotation(yaw * 180 / M_PI);
}

void CustomScene::display_roi(std::vector<std::pair<float, float>> points){
    //this->clear();
    static std::vector<QGraphicsLineItem *> line_items;
    for (int i = 0; i < line_items.size(); i++){
        line_items.at(i)->~QGraphicsLineItem();
    }
    line_items.clear();
    if (points.size() > 0){
        for (int i = 0; i < points.size() - 1; i++){
            line_items.push_back(
                this->addLine(points.at(i).first, points.at(i).second,
                              points.at(i + 1).first, points.at(i + 1).second));
        }
        line_items.push_back(
            this->addLine(
                points.at(points.size() - 1).first, points.at(points.size() - 1).second,
                points.at(0).first, points.at(0).second));
    }
}
void CustomScene::display_visual_points(std::vector<std::pair<float, float>> points, bool clear_flag){
    //this->clear();
    static std::vector<QGraphicsLineItem *> line_items;
    if (clear_flag){
        for (int i = 0; i < line_items.size(); i++){
            //line_items.at(i)->
            //this->removeItem(line_items.at(i));
            //line_items.at(i)->~QGraphicsLineItem();
            delete line_items.at(i);
        }
        //this->clear();
        line_items.clear();
    }
    if (points.size() > 3){
        for (int i = 2; i < points.size() - 3; i++){
            if (clear_flag){
                line_items.push_back(
                    this->addLine(points.at(i).first, points.at(i).second,
                                  points.at(i + 1).first, points.at(i + 1).second,
                                  QColor(255, 0, 0)));
            }else{
                line_items.push_back(
                    this->addLine(points.at(i).first, points.at(i).second,
                                  points.at(i + 1).first, points.at(i + 1).second,
                                  QColor(0, 255, 0)));
            }
        }
    }
}
void CustomScene::display_sonar_obs(std::vector<std::pair<float, float>> points){
    static std::vector<QGraphicsRectItem *> rect_items;
    for (int i = 0; i < rect_items.size(); i++){
        rect_items.at(i)->~QGraphicsRectItem();
    }
    rect_items.clear();
    if (points.size() > 0){
        for (int i = 0; i < points.size(); i++){
            rect_items.push_back(
                        this->addRect(points.at(i).first,
                                      points.at(i).second,
                                      5, 5, QColor(255, 0, 0)));
        }
    }
}
void CustomScene::display_freespace_obs(std::vector<std::pair<float, float>> points){
    static std::vector<QGraphicsRectItem *> rect_items;
    for (int i = 0; i < rect_items.size(); i++){
        rect_items.at(i)->~QGraphicsRectItem();
    }
    rect_items.clear();
    if (points.size() > 0){
        for (int i = 0; i < points.size(); i++){
            rect_items.push_back(
                        this->addRect(points.at(i).first,
                                      points.at(i).second,
                                      5, 5, QColor(0, 255, 0)));
        }
    }
}
void CustomScene::display_visual_obs(std::vector<std::pair<float, float>> blpoints,
                                     std::vector<std::pair<float, float>> tlpoints,
                                     std::vector<std::pair<float, float>> trpoints,
                                     std::vector<std::pair<float, float>> brpoints){
    static std::vector<QGraphicsLineItem *> line_items;
    if (1){
        for (int i = 0; i < line_items.size(); i++){
            line_items.at(i)->~QGraphicsLineItem();
        }
        line_items.clear();
    }
    if (blpoints.size() > 0){
        for (int i = 0; i < blpoints.size(); i++){
                line_items.push_back(
                    this->addLine(blpoints.at(i).first, blpoints.at(i).second,
                                  tlpoints.at(i).first, tlpoints.at(i).second,
                                  QColor(0, 0, 0)));
                line_items.push_back(
                    this->addLine(tlpoints.at(i).first, tlpoints.at(i).second,
                                  trpoints.at(i).first, trpoints.at(i).second,
                                  QColor(0, 0, 0)));
                line_items.push_back(
                    this->addLine(trpoints.at(i).first, trpoints.at(i).second,
                                  brpoints.at(i).first, brpoints.at(i).second,
                                  QColor(0, 0, 0)));
                line_items.push_back(
                    this->addLine(brpoints.at(i).first, brpoints.at(i).second,
                                  blpoints.at(i).first, blpoints.at(i).second,
                                  QColor(0, 0, 0)));
        }
    }
}
