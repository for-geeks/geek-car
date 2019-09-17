#ifndef EXERCISE1_H
#define EXERCISE1_H
#include "share_mem/share_memory.h"
#include "control_button.h"
#include "list_view_standard.h"
#include <QtWidgets/QWidget>
#include <QtCore/QTimer>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtGui/QPainter>
#include "ui_exercise1.h"
#include <QtCore/QThread>
#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
#include <iostream>
#include <QtGui/QPixmap>
#include "umb_standard.h"
#include <QGraphicsPixmapItem>
//extern exercise1 *exercise1_ptr;
//extern int *exercise1_ptr = nullptr;
#include <QCloseEvent>
#include "box.h"
extern int* exercise1_ptr;
typedef struct _image_t{
    long type;
    unsigned char data[900 * 900];
    int width;
    int height;
}image_t;
typedef struct _compress_image_t{
    int length;
    int width;
    int height;
    unsigned char data[20000];

}compress_image_t;
typedef struct _Point_t{
    double x;
    double y;
    double z;
}Point_t;
typedef struct _Point4_t{
    double x;
    double y;
    double z;
    double w;
}Point4_t;
typedef struct _pose_t{
    Point_t velocity;
    Point4_t rotation;
    Point_t angular_velocity;
    Point_t angular_acceleration;
    uint8_t tracker_confidence;
    uint8_t mapper_confidence;
    Point_t translation;
}pose_t;
void send_image2qt(unsigned char * location, int width, int height, int size);
class MessageCom : public QThread
{
    Q_OBJECT

public:
    QObject *parenttype;
    void msg_init(void);
    int msqid;
    image_t image_reader;

    CShareMemory *csm;
    explicit MessageCom(QObject *parent = 0) : QThread(parent)
    {
        msg_init();
        csm = new CShareMemory("txh", sizeof(image_t) * 2);
        //init();
    }
    ~MessageCom(void){
        stop();
        requestInterruption();
        wait();
    };
    void init()
    {
        start(HighestPriority);

    }
    void stop(){
        quit();
    }
    void read_image(void){
        csm->GetFromShareMem((u8 *)&image_reader, sizeof(image_reader));
        send_image2qt(image_reader.data, image_reader.width, image_reader.height, image_reader.type);
    }
private:

protected:
    void run(){
        //static int count = 0;
        while (!QThread::currentThread()->isInterruptionRequested()){

        }
    }
};


namespace Ui {
class exercise1;
}

class exercise1 : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    explicit exercise1(QWidget *parent = 0);

    ~exercise1();
    void update_image(unsigned char * location, int width, int height, int size);
    QImage *raw_image = nullptr;
    void update_pose(unsigned char * buff, unsigned int lengt);
    void update_image_trans(unsigned char * buff, unsigned int lenth);

    MessageCom msgcom;
    ControlButton * control;
    QPixmap pix;
    QPixmap pix_trans;
private:
    Ui::exercise1 *ui;
    pose_t pose_reader;
    UmbStandard *umb_center;
    QTimer *qtimer;
    VowelCube *cube;

    ListViewStandard *listview;// = new ListViewStandard(ui->treeWidget);

    void _display_altitude(void);
private slots:
    void _timer_update(void);

    void on_exercise1_destroyed();
protected:
     void closeEvent(QCloseEvent *event);
};

#endif // EXERCISE1_H
