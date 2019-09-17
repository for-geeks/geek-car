#ifndef EXERCISE2_H
#define EXERCISE2_H

#include <QWidget>
#include "list_view_standard.h"
#include <QtWidgets/QWidget>
#include <QtCore/QTimer>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include "ui_exercise2.h"
#include <QtCore/QThread>
#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
#include <iostream>
#include "umb_standard.h"
#include "oscilloscope.h"
#include "list_view_standard.h"
extern int* exercise2_ptr;
typedef struct _chassie_s{
    float steer;
    float throttle;
    float speed;
}chassie;
namespace Ui {
class exercise2;
}

class exercise2 : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    explicit exercise2(QWidget *parent = 0);
    ~exercise2();
    void update_chassie_info(char* buff, int len){
        if (len == 12){
            memcpy(&chassie_info, buff, len);
        }
    }

private:
    Ui::exercise2 *ui;
    QTimer *qtimer;
    UmbStandard *umb_center;
    chassie chassie_info;
    ListViewStandard *listview;// = new ListViewStandard(ui->treeWidget);
    Oscilloscope *oscilloscope;
private slots:
    void timer_update(void);
    void on_radioButton_toggled(bool checked);

protected:
     void closeEvent(QCloseEvent *event);
};

#endif // EXERCISE2_H
