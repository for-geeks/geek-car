#-------------------------------------------------
#
# Project created by QtCreator 2019-06-21T11:02:24
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = apollo_edu_terminal
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        $$PWD/gui_core/*.cpp \
        $$PWD/gui_core/scene/*.cpp \
        gui_core/exercise1/exercise1.cpp \
        $$PWD/share_mem/*.cpp \
        $$PWD/gui_core/list_view_standard/*.cpp \
        $$PWD/gui_core/control_button/*.cpp \
        $$PWD/gui_core/rotation_gl/*.cpp \
        gui_core/exercise2/exercise2.cpp \
        gui_core/exercise_lateral_control/lateral_control_exercise.cpp \
        gui_core/exercise_view_caliberation/viewcaliberationexercise.cpp \
        gui_core/exercise_view_caliberation/lanedetectionexercise.cpp \
        gui_core/exercise_view_caliberation/localization.cpp \


HEADERS += \
        $$PWD/gui_core/*.h \
        $$PWD/share_mem/*.h \
        gui_core/exercise1/exercise1.h \
        $$PWD/gui_core/list_view_standard/*.h \
        #gui_core/umb_standard.h \
        $$PWD/gui_core/control_button/*.h \
        $$PWD/gui_core/rotation_gl/*.h \
        gui_core/exercise2/exercise2.h \
        $$PWD/gui_core/oscilloscope/*.h \
        gui_core/exercise_lateral_control/lateral_control_exercise.h \
        gui_core/exercise_view_caliberation/viewcaliberationexercise.h \
        gui_core/exercise_view_caliberation/lanedetectionexercise.h \
        gui_core/exercise_view_caliberation/localization.h \
        $$PWD/gui_core/scene/*.h \
        $$PWD/gui_core/least_square/*.h \

FORMS += \
        $$PWD/gui_core/*.ui \
        gui_core/exercise1/exercise1.ui \
        gui_core/exercise2/exercise2.ui \
        gui_core/exercise_lateral_control/lateral_control_exercise.ui \
        gui_core/exercise_view_caliberation/viewcaliberationexercise.ui \
        gui_core/exercise_view_caliberation/lanedetectionexercise.ui \
        gui_core/exercise_view_caliberation/localization.ui

INCLUDEPATH += \
                /usr/local/include \
                /usr/include \
                /usr/local/include/openssh \
                /usr/include/qtermwidget5 \
                /home/raosiyue/libssh2/src \
                gui_core/exercise1 \
                gui_core/exercise2 \
                gui_core \
                $$PWD/share_mem \
                $$PWD/gui_core/list_view_standard \
                $$PWD/gui_core/control_button \
                $$PWD/gui_core/rotation_gl \
                /home/raosiyue/umb/umb_output/include \
                $$PWD/gui_core/oscilloscope \
                gui_core/exercise_lateral_control \
                gui_core/exercise_view_caliberation \
                $$PWD/gui_core/scene \
                $$PWD/gui_core/least_square \

LIBS += \
        -L/usr/local/lib -lqtermwidget5 \
        -L/home/raosiyue/umb/rti_connext_dds-5.3.1/lib/x64Linux3.xgcc4.6.3 -lnddsc -lnddscore \
        -L/home/raosiyue/umb/umb_output/lib -lumb -ldl -lnsl -lm -lrt -lglut \
