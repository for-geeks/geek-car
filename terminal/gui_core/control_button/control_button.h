#ifndef CONTROL_BUTTON_H
#define CONTROL_BUTTON_H
#include <vector>
#include <string>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QKeyEvent>
#include <QWidget>
#include <QTimer>
#include <iostream>
class ControlButton: public QWidget{
    Q_OBJECT
public:
    explicit ControlButton(QGraphicsView * graphicsView){
        this->graphicsView = graphicsView;
        virtual_press_button_init();
        timer = new QTimer(this);
        this->connect(timer, SIGNAL(timeout()), this, SLOT(update_key_value()));
        timer->start(50);

    }
    ~ControlButton(){}
    float throttle_cmd;
    float brake_cmd;
    float steer_cmd;
private:
    QTimer *timer;
    QGraphicsView * graphicsView;
    //void update_key_value(void);
    bool s_w_pressflag = false;
    bool s_s_pressflag = false;
    bool s_a_pressflag = false;
    bool s_d_pressflag = false;
    QGraphicsRectItem * s_myRect1;
    QGraphicsRectItem * s_myRect2;
    QGraphicsRectItem * s_myRect3;
    QGraphicsRectItem * s_myRect4;

    void virtual_press_button_init();
    void virtual_press_button_update(void);
protected:
    virtual void keyPressEvent(QKeyEvent *ev);
    virtual void keyReleaseEvent(QKeyEvent *ev);
private slots:
    void update_key_value(void);
};
#endif
