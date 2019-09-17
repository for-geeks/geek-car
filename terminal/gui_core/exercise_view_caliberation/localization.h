#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include "umb_standard.h"
#include "exercise1.h"
#include <QWidget>
#include "obs_item.h"
#include <QWidget>
#include "lateral_control_exercise.h"
extern int* Localization_ptr;
namespace Ui {
class Localization;
}

typedef struct _position_s{
    double x;
    double y;
    double z;
    double yaw;

}position_s;

typedef struct _localization_s{
    position_s apriltag0;
    position_s apriltag1;
    position_s apriltag;
    position_s prediction;

}localization_s;

class Localization : public QWidget
{
    Q_OBJECT
signals:
    void close_sig(int);
public:
    explicit Localization(QWidget *parent = 0);
    ~Localization();
    //update_localiztion(cahr);
    void update_localiztion(char* buff, int len){
        if (len == sizeof(localization)){
            memcpy(&localization, buff, len);
            _log_apriltag();
            _log_predict();

            //_log_apriltag();
            //_log_predict();
        }
    }
    void update_kalman(char* buff, int len){
        if (len == sizeof(position_s)){
            memcpy(&kalman_pos, buff, len);
            _log_kalman();
        }
    }
    ControlButton * control;
private:
    void _log_apriltag(void);
    void _log_predict(void);
    void _log_kalman(void);
    Ui::Localization *ui;
    position_s kalman_pos;
    localization_s localization;
    std::vector<trajectory_t> traj;
    std::vector<trajectory_t> traj_pre;
    std::vector<std::pair<float, float>> kalman_vec;
    trajectory_t start_point;
    std::vector<std::pair<float, float>> traj_vec;
    std::vector<std::pair<float, float>> traj_pre_vec;
    pose_t pose_reader;

    UmbStandard *umb_center;
    QTimer *qtimer;
    CustomScene *scene;
    CustomScene *scene_lane;

    LineFitter fitter;
    float last_idx = -1;
    float this_idx = -1;
    chassie chassie_info;
    trajectory_t point;
private slots:
    void _timer_update(void);
protected:
     void closeEvent(QCloseEvent *event);

};

#endif // LOCALIZATION_H
