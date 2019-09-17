#ifndef VIEWCALIBERATIONEXERCISE_H
#define VIEWCALIBERATIONEXERCISE_H
#include "exercise1.h"
#include "obs_item.h"
#include "lateral_control_exercise.h"
#include <QWidget>
extern int* ViewCaliberationExercise_ptr;
namespace Ui {
class ViewCaliberationExercise;
}

class ViewCaliberationExercise : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    explicit ViewCaliberationExercise(QWidget *parent = 0);
    ~ViewCaliberationExercise();
    void update_trajectory(char* buff, int len){
        memset(trajectory_pts_in, 0, 50 * sizeof(Point2d_s));
        if (len <= 50 * sizeof(Point2d_s)){
            memcpy(trajectory_pts_in, buff, len);
        }else{
            memcpy(trajectory_pts_in, buff, 50 * sizeof(Point2d_s));
        }

    }
private:
    exercise1 *exercise01 = nullptr;
    QTimer *qtimer;
    Ui::ViewCaliberationExercise *ui;
    CustomScene *scene_origin;
    CustomScene *scene_transform;
    QGraphicsPixmapItem *pic_item_origin;
    QGraphicsPixmapItem *pic_item_trans;
    ListViewStandard *listview;
    UmbStandard *umb_center;
    Point2d_s trajectory_pts[50];
    Point2d_s trajectory_pts_in[50];

protected:
     void closeEvent(QCloseEvent *event);
private slots:
     void _timer_update(void);
     void on_pushButton_2_clicked();
};

#endif // VIEWCALIBERATIONEXERCISE_H
