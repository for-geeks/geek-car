#include "control_button.h"
#include <QKeyEvent>
#include <iostream>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>

void ControlButton::update_key_value(void){
    if (s_w_pressflag == true){
        throttle_cmd = 0.9 * throttle_cmd + 0.1 * 12;
    }
    if (s_s_pressflag == true){
        throttle_cmd = 0.9 * throttle_cmd + 0.1 * -12;
    }
    if ((s_s_pressflag == false) && (s_w_pressflag == false)){
        throttle_cmd = 0.1 * brake_cmd + 0.1 * 0;
    }
    if (s_a_pressflag == true){
        steer_cmd = 0.8 * steer_cmd + 0.2 * 60;
    }else{
        //steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    if (s_d_pressflag == true){
        steer_cmd = 0.8 * steer_cmd + 0.2 * -60;
    }else{
        //steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    if ((s_d_pressflag == false) && (s_a_pressflag == false)){
        steer_cmd = 0.1 * steer_cmd + 0.1 * 0.0;
    }
    virtual_press_button_update();
    //throttle_cmd = 0.9 * throttle_cmd + 0.1 * 0.3;
}
void ControlButton::keyPressEvent(QKeyEvent *ev)
{
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
    return;
}
void ControlButton::keyReleaseEvent(QKeyEvent *ev)
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
    return;
}
void ControlButton::virtual_press_button_init(void){
    QGraphicsView * wid_ptr = graphicsView;
    QGraphicsScene *scene = new QGraphicsScene();
    scene->setSceneRect(0, 0, wid_ptr->width(), wid_ptr->height());
    wid_ptr->setScene(scene);
    this->setFocusPolicy(Qt::ClickFocus);
    this->grabKeyboard();
    //scene->setBackgroundBrush(QPixmap(":/images/background.png"));
    s_myRect1 = scene->addRect(100, 50, 50, 50);
    s_myRect1->setBrush(QColor(0, 255, 0));
    s_myRect2 = scene->addRect(100, 120, 50, 50);
    s_myRect2->setBrush(QColor(0, 255, 0));
    s_myRect3 = scene->addRect(30, 120, 50, 50);
    s_myRect3->setBrush(QColor(0, 255, 0));
    s_myRect4 = scene->addRect(170, 120, 50, 50);
    s_myRect4->setBrush(QColor(0, 255, 0));
}
void ControlButton::virtual_press_button_update(void){
    QGraphicsView * wid_ptr = graphicsView;
    s_myRect1->setBrush(QColor(255 * s_w_pressflag, 255, 0));
    s_myRect2->setBrush(QColor(255 * s_s_pressflag, 255, 0));
    s_myRect3->setBrush(QColor(255 * s_a_pressflag, 255, 0));
    s_myRect4->setBrush(QColor(255 * s_d_pressflag, 255, 0));
}
