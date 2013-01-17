#ifndef GLRENDERTHREAD_H
#define GLRENDERTHREAD_H

#include <QThread>
#include <QtGui/QCursor>
#include <vector>
#include <iostream>
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
    void updateZoom (float zoom);

    void setPointCloud (vector<float> &point_cloud);
    vector<float> _point_cloud;

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
    QPoint _current_mouse_pos;
    float _zoom;

signals:
public slots:
};

#endif // GLRENDERTHREAD_H
