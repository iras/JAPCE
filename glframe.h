#ifndef GLFRAME_H
#define GLFRAME_H

#include <QGLWidget>
#include "glrenderthread.h"
#include <iostream>
#include <vector>

#include <pointcloud.h>

#include <opencv2/core/core.hpp>
using namespace std;

class GLFrame : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLFrame (QWidget *parent = 0);
    ~GLFrame ();

    void initRenderThread (void);
    void stopRenderThread (void);
    void resumeRenderThread (void);
    void pauseRenderThread (void);

    bool getCtrlMetaFlag ();
    bool getAltFlag ();

    void delegateSetPointCloud (PointCloud *point_cloud);
    void DelegateCameraPyramidAddition  (cv::Mat_<double> &camera_matrix, int rows, int cols);

signals:

public slots:

protected:
    void closeEvent  (QCloseEvent *event);
    void resizeEvent (QResizeEvent *event);
    void paintEvent  (QPaintEvent *);
    void focusInEvent  (QFocusEvent *);
    void focusOutEvent (QFocusEvent *);
    void wheelEvent  (QWheelEvent *);
    void keyPressEvent   (QKeyEvent *event);
    void keyReleaseEvent (QKeyEvent *event);

    GLRenderThread RenderThread;
    bool Ctrl_or_Meta_key_pressed;
    bool Alt_key_pressed;

private:

};

#endif // GLFRAME_H
