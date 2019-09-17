#ifndef LANEDETECTIONEXERCISE_H
#define LANEDETECTIONEXERCISE_H
#include "umb_standard.h"
#include "exercise1.h"
#include <QWidget>
#include "obs_item.h"

typedef struct _Coefficient_t{
    double a;
    double b;
    double c;
}Coefficient_t;

typedef struct _Laneinfo_s{
  Coefficient_t left_lane;
  Coefficient_t right_lane;
  Coefficient_t left_lane_view;
  Coefficient_t right_lane_view;
}Laneinfo_s;

extern int* LaneDetectionExercise_ptr;
namespace Ui {
class LaneDetectionExercise;
}

class LaneDetectionExercise : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    explicit LaneDetectionExercise(QWidget *parent = 0);
    ~LaneDetectionExercise();
    void update_lane(char* buff, int len){
        if (len == sizeof(laneinfo)){
            memcpy(&laneinfo, buff, len);
        }
    }
private:
    UmbStandard *umb_center;
    QTimer *qtimer;
    Laneinfo_s laneinfo;
    CustomScene *scene;
    Ui::LaneDetectionExercise *ui;
    ListViewStandard *listview;// = new ListViewStandard(ui->treeWidget);
protected:
     void closeEvent(QCloseEvent *event);
private slots:
     void _time_update(void);
};

#endif // LANEDETECTIONEXERCISE_H
