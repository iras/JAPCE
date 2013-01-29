#ifndef GLRENDERTHREAD_H
#define GLRENDERTHREAD_H

#include <QThread>
#include <QtGui/QCursor>
#include <QtGui/QVector3D>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/glut.h"

using namespace std;

class GLFrame;
class QSize;


class GLRenderThread : public QThread
{
    Q_OBJECT

public:
    explicit GLRenderThread (GLFrame *parent = 0);
    void resizeViewport (const QSize &size);
    void run  (void);
    void stop (void);
    void updateCameraDistanceFromCenter (float zoom);

    vector<float> _point_cloud;
    void setPointCloud (vector<float> &point_cloud);
    void addCameraPyramid  (cv::Mat_<double> &camera_matrix, int rows, int cols);

protected:
    void GLInit   (void);
    void GLResize (int width, int height);
    void paintGL  (void);

private:
    bool doRendering, doResize;
    int w, h, FrameCounter;

    GLFrame *GLFrame_var;
	QCursor _mouse;
    QPoint _last_pos;
    QPoint _delta;
    QPoint _current_angles;
    QVector3D _o;
    float _zoom;
    GLfloat m[16];

    void drawGrid (void);
    void drawCurrentOriginAxes (void);
    void displayCameraPyramids (void);

    vector<vector<cv::Mat_<double> > > camera_pyramids;

signals:
public slots:
};

#endif // GLRENDERTHREAD_H
