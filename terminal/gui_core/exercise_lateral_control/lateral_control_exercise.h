#ifndef LATERAL_CONTROL_EXERCISE_H
#define LATERAL_CONTROL_EXERCISE_H

#include <QWidget>
#include <QTimer>
#include "umb_standard.h"
#include "exercise1.h"
#include "obs_item.h"
#include "least_square.h"
#include "exercise2.h"
//#include "localization.h"
typedef struct _trajectory_t{
    double x;
    double y;
    double yaw;
}trajectory_t;

typedef struct _laneinfo_t{
    double a;
    double b;
    double c;
}laneinfo_t;

typedef struct _Point2d_s {
  // meters
  double x;
  double y;
}Point2d_s;

typedef struct _PlanningInfo_s {
  Point2d_s start_point;
  Point2d_s end_point;
  Point2d_s obs_points;
}PlanningInfo_s;

extern int* lateral_control_exercise_ptr;

namespace Ui {
class lateral_control_exercise;
}

class lateral_control_exercise : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    int number = 8;
    explicit lateral_control_exercise(QWidget *parent = 0);
    ~lateral_control_exercise();
    void update_pose(unsigned char * buff, unsigned int lengt);
    void update_chassie_info(char* buff, int len){
        if (len == 12){
            memcpy(&chassie_info, buff, len);
        }
    }
    void set_exe_number(int num){
        number = num;
    }
    void update_trajectory(char* buff, int len){
        memset(trajectory_pts, 0, 50 * sizeof(Point2d_s));
        if (len <= 50 * sizeof(Point2d_s)){
            memcpy(trajectory_pts, buff, len);
        }else{
            memcpy(trajectory_pts, buff, 50 * sizeof(Point2d_s));
        }

    }
    void update_obs(char* buff, int len){
       // memcpy(&obs_pos, buff, len);
    }

    ControlButton * control;
private:
    //position_s obs_pos;
    Point2d_s trajectory_pts[50];
    Ui::lateral_control_exercise *ui;
    std::vector<trajectory_t> traj;
    trajectory_t start_point;
    std::vector<std::pair<float, float>> traj_vec;
    pose_t pose_reader;
    PlanningInfo_s target;
    UmbStandard *umb_center;
    QTimer *qtimer;
    CustomScene *scene;
    CustomScene *scene_lane;
    LineFitter fitter;
    float last_idx = -1;
    float this_idx = -1;
    chassie chassie_info;
    trajectory_t point;
    void _display_position(void);
    void _log_trajectory(void);
    double _distance_2d(trajectory_t t1, trajectory_t t2);
    std::vector<std::pair<float, float>> _vehicle2global_asvec(
            std::vector<std::pair<float, float>> invec, bool mode);
    void _vehicle2global(float x, float y, float * x_trans, float * y_trans, bool mode);

private slots:
    void _timer_update(void);
    void on_checkBox_record_toggled(bool checked);
    unsigned int _find_nearest_idx(float x, float y);
    std::vector<std::pair<float, float>> _sense_points(void);
    void _calculate_lane(laneinfo_t *lane);
    std::vector<std::pair<float, float>> _global2vehicle_asvec(
            std::vector<std::pair<float, float>> invec);
    void _global2vehicle(float x, float y, float * x_trans, float * y_trans);

    void on_pushButton_2_clicked();

protected:
     void closeEvent(QCloseEvent *event);
};

#endif // LATERAL_CONTROL_EXERCISE_H



