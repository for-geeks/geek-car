#ifndef OSCILLOSCOPE_H
#define OSCILLOSCOPE_H
#include <iostream>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <QTimer>
#include <exception>
#include <QDateTime>
#include <fstream>
class Oscilloscope{
public:
    Oscilloscope(QGraphicsView *graphic_view){
        wid_ptr = graphic_view;
    }
    ~Oscilloscope(){}
    void init(double scale_init){
        QGraphicsScene *scene = new QGraphicsScene();
        scene->setSceneRect(0, 0, wid_ptr->width() * 0.9, wid_ptr->height() * 0.9);
        wid_ptr->setScene(scene);
        for (int u = 0; u < scene->height(); u++)
        {
            if (u % 30 == 0)
            {
                if (u == 0){
                    scene->addLine(10, scene->height() - (u + 10),
                        scene->width(), scene->height() - (u + 10), QPen(QColor(255, 0, 0)));
                }else{
                    scene->addLine(10, scene->height() - (u + 10),
                        scene->width(), scene->height() - (u + 10), QPen(QColor(0, 0, 0)));
                }
                float v = u / 100.0;
                QGraphicsTextItem * text_item = scene->addText(QString::number(v).sprintf("%2.1f", v));
                text_item->setPos(0 - 20, scene->height() - (u + 30));
            }
        }
        //绘制竖直方向的直线
        for (int t = 0; t < scene->width(); t++)
        {
            if (t % 30 == 0)
            {
                if (t == 0){
                    scene->addLine(t + 10, scene->height() - 10, t + 10,
                        scene->height() - scene->height() + 10, QPen(QColor(255, 0, 0)));
                }else{
                    scene->addLine(t + 10, scene->height() - 10, t + 10,
                        scene->height() - scene->height() + 10, QPen(QColor(0, 0, 0)));
                }
            }
        }
    }

    void update(float value, float value2, QString val_name, float scale = 1, float offset = 0){
        static bool start_flag = false;
        static unsigned int time_cursor = 0;
        //QGraphicsView * wid_ptr = ui->can_test_value_viewer_2;
        QGraphicsScene *scene = wid_ptr->scene();
        static float last_x = 10;
        static float last_y = scene->height() - 10;
        static float last_x2 = 10;
        static float last_y2 = scene->height() - 10;
        static QGraphicsTextItem * value_name;
        if (!start_flag){
            value_name = scene->addText(QString("test"));
            start_flag = true;
        }
        value = value * scale +
                offset;
        value2 = value2 * scale +
                offset;
        if (value < 0){
            value = 0;
        }
        value_name->setDefaultTextColor(QColor(0, 0, 255));
        value_name->setPlainText(val_name);
        value_name->setPos(scene->width() * 0.4, scene->height() * 0.05);
        if (time_cursor >= scene->width()){
            scene->clear();
            init(scale);
            value_name = scene->addText(val_name);
            time_cursor = 0;
            last_y = scene->height() - 10;
            last_x = 10;
            last_x2 = 10;
            last_y2 = scene->height() - 10;
            value_name->setPlainText(val_name);
            start_flag = false;
        }else{
            scene->addLine(last_x,
                last_y, 10 + time_cursor, scene->height() - (value + 10),
                QPen(QColor(0, 0, 255)));
            scene->addLine(last_x2,
                last_y2, 10 + time_cursor, scene->height() - (value2 + 10),
                QPen(QColor(255, 0, 0)));
            //line_ptr->se
            last_x = 10 + time_cursor;
            last_x2 = 10 + time_cursor;
            last_y = scene->height() - (value + 10);
            last_y2 = scene->height() - (value2 + 10);
            time_cursor++;
        }
    }
private:
    QGraphicsView * wid_ptr;
};





#endif
