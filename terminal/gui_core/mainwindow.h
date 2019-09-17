#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "qtermwidget5/qtermwidget.h"
#include "gui_core/exercise1/exercise1.h"
#include "ui_mainwindow.h"
#include "exercise2.h"
#include "lateral_control_exercise.h"
#include "lanedetectionexercise.h"
#include "localization.h"
#include "viewcaliberationexercise.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void close_exercise(int num);
    void on_exercise1_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_8_clicked();

private:
    Ui::MainWindow *ui;
    QTermWidget *console;
    exercise1 *exercise01 = nullptr;
    exercise2 *exercise02 = nullptr;
    lateral_control_exercise *lateral_exercise = nullptr;
    LaneDetectionExercise *lane_exercise = nullptr;
    Localization *localization_exercise = nullptr;
    ViewCaliberationExercise *viewcalib_exercise = nullptr;
    void _initterminal(void);
};

#endif // MAINWINDOW_H
