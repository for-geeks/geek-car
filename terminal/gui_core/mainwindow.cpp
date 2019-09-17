#include "mainwindow.h"

//#include "widget.h"
#include <QtWidgets/QApplication>
#include <QtCore/QDebug>
//#include <QApplication>
#include <QtWidgets/QLayout>
// Add QDesktopServices
#include <QtGui/QDesktopServices>
#include <QtGui/QKeySequence>
#include <QtWidgets/QMainWindow>
//#include <QTermWidget>
#include "qtermwidget5/qtermwidget.h"
#include <unistd.h>
#include "exercise1.h"
#include "exercise2.h"
#include "lateral_control_exercise.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{


    ui->setupUi(this);
    _initterminal();
    ui->ssh_iplineEdit->setText(QString("10.42.0.1"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::_initterminal(void){
      //QApplication app(argc, argv);
      //Widget w;
      //
      //QMainWindow *mainWindow = new QMainWindow();
      setenv("TERM", "konsole-256color", 1);

      console = new QTermWidget();

      QFont font = QApplication::font();
      font.setFamily("Monospace");
      font.setPointSize(12);

      console->setTerminalFont(font);
      console->setColorScheme("Solarized");
      console->setTerminalOpacity(0.9);

      // Here we connect it all together
    #if 0
      QObject::connect(console, &QTermWidget::urlActivated, mainWindow,
                       activateUrl);

      QObject::connect(console, &QTermWidget::termKeyPressed, mainWindow,
                       [=](const QKeyEvent *key) -> void {
                         if (key->matches(QKeySequence::Copy)) {
                           console->copyClipboard();
                         } else if (key->matches(QKeySequence::Paste)) {
                           console->pasteClipboard();
                         }
                       });
    #endif
      QObject::connect(console, SIGNAL(finished()), this, SLOT(close()));
      console->setScrollBarPosition(QTermWidget::ScrollBarRight);

      //this->setCentralWidget(console);

      console->setFixedSize(QSize(640, 480));
      console->setColorScheme(QString("GreenOnBlack"));
      qDebug() << console->availableColorSchemes();
      this->layout()->addWidget(console);
      console->sendText(QString("cd \n"));
      //this->ui->dockWidgetContents_3 = console;
      //this->ui->widget = console;
      //ui->widget->show();
      //console->setFixedSize();
      //this->setCentralWidget();
      //w.set
      //mainWindow->resize(600, 400);
      //mainWindow->show();
      //return app.exec();
}

void MainWindow::on_pushButton_clicked()
{
    QStringList list;
    //console->setArgs(list << QString("ls"));
    console->sendText(QString("ssh raosiyue@") + ui->ssh_iplineEdit->text() + QString("\n"));
    //sleep(1);
    //console->sendText(QString("123456\n"));
}

void MainWindow::on_exercise1_pushButton_clicked()
{
    if (exercise01 != nullptr){
        exercise01->~exercise1();
    }

    exercise01 = new exercise1();
    connect(exercise01, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    exercise01->show();
    exercise1_ptr = (int*)exercise01;
}

void MainWindow::close_exercise(int num){
    switch (num){
        case 1:
            exercise1_ptr = nullptr;
            exercise01->setFocusPolicy(Qt::NoFocus);
            exercise01->control->releaseKeyboard();
            exercise01->~exercise1();
            std::cout << "exercise1 closed" << std::endl;
            exercise01 = nullptr;
            break;
        case 2:
            ViewCaliberationExercise_ptr = nullptr;
            viewcalib_exercise->~ViewCaliberationExercise();
            std::cout << "exercise2 closed" << std::endl;
            viewcalib_exercise = nullptr;
            break;
        case 3:
            LaneDetectionExercise_ptr = nullptr;
            lane_exercise->~LaneDetectionExercise();
            std::cout << "exercise3 closed" << std::endl;
            lane_exercise = nullptr;
            break;
        case 4:
            Localization_ptr = nullptr;
            localization_exercise->~Localization();
            std::cout << "exercise4 closed" << std::endl;
            localization_exercise = nullptr;
            break;
        case 7:
            exercise2_ptr = nullptr;
            exercise02->~exercise2();
            std::cout << "exercise2 closed" << std::endl;
            exercise02 = nullptr;
            break;
        case 8:
            lateral_control_exercise_ptr = nullptr;
            lateral_exercise->control->releaseKeyboard();
            delete lateral_exercise;
            //lateral_exercise->~lateral_control_exercise();
            std::cout << "exercise8 closed" << std::endl;
            lateral_exercise = nullptr;
            break;
        default:
            break;
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    if (exercise02 != nullptr){
        exercise02->~exercise2();
    }

    exercise02 = new exercise2();
    connect(exercise02, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    exercise02->show();
    exercise2_ptr = (int*)exercise02;
}

void MainWindow::on_pushButton_3_clicked()
{
    if (lateral_exercise != nullptr){
        lateral_exercise->~lateral_control_exercise();
    }
    lateral_exercise = new lateral_control_exercise();
    connect(lateral_exercise, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    lateral_exercise->show();
    lateral_control_exercise_ptr = (int*)lateral_exercise;
}

void MainWindow::on_pushButton_4_clicked()
{
    if (viewcalib_exercise != nullptr){
        viewcalib_exercise->~ViewCaliberationExercise();
    }
    viewcalib_exercise = new ViewCaliberationExercise();
    connect(viewcalib_exercise, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    viewcalib_exercise->show();
    ViewCaliberationExercise_ptr = (int*)viewcalib_exercise;
}

void MainWindow::on_pushButton_5_clicked()
{
    if (lane_exercise != nullptr){
        lane_exercise->~LaneDetectionExercise();
    }
    lane_exercise = new LaneDetectionExercise();
    connect(lane_exercise, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    lane_exercise->show();
    LaneDetectionExercise_ptr = (int*)lane_exercise;
}

void MainWindow::on_pushButton_6_clicked()
{
    if (localization_exercise != nullptr){
        localization_exercise->~Localization();
    }
    localization_exercise = new Localization();
    connect(localization_exercise, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    localization_exercise->show();
    Localization_ptr = (int*)localization_exercise;
}

void MainWindow::on_pushButton_8_clicked()
{
    if (lateral_exercise != nullptr){
        lateral_exercise->~lateral_control_exercise();
    }
    lateral_exercise = new lateral_control_exercise();
    lateral_exercise->set_exe_number(6);
    connect(lateral_exercise, SIGNAL(close_sig(int)), this, SLOT(close_exercise(int)));
    lateral_exercise->show();
    lateral_control_exercise_ptr = (int*)lateral_exercise;
}
