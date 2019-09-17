#ifndef VOWELCUBE_H
#define VOWELCUBE_H

#include <QtOpenGL/QGLWidget>
#include <QRadialGradient>

//使用OpenGL绘制立方体，使用QPainter绘制背景
//的渐变，接着使用renderText（）绘制立方体角上的8个
//元音字母，最后使用QPainter和QTextDocument绘制图例。
//用户可以单击并拖动鼠标来旋转立方体，并且可以使用鼠标滚轮进行放大或缩小
class VowelCube : public QGLWidget
{
    Q_OBJECT

public:
    VowelCube(QWidget *parent = 0);
    ~VowelCube();
    void set_rotation(float x, float y, float z);
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    void createGradient();
    void createGLObject();
    void drawBackground(QPainter *painter);
    void drawCube();
    void drawLegend(QPainter *painter);

    GLuint glObject;
    QRadialGradient gradient;
    GLfloat rotationX;
    GLfloat rotationY;
    GLfloat rotationZ;
    GLfloat scaling;
    QPoint lastPos;
};

#endif
