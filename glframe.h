#ifndef GLFRAME_H
#define GLFRAME_H

#include <QGLWidget>
#include "glrenderthread.h"
#include <iostream>
#include <vector>
using namespace std;

class GLFrame : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLFrame (QWidget *parent = 0);
    ~GLFrame ();
    void initRenderThread (void);
    void stopRenderThread (void);

    bool getCtrlMetaFlag ();
    void displayPointCloud (vector<float> &point_cloud);

signals:

public slots:

protected:
    void closeEvent  (QCloseEvent *event);
    void resizeEvent (QResizeEvent *event);
    void paintEvent  (QPaintEvent *);
    void wheelEvent  (QWheelEvent *);
    void keyPressEvent (QKeyEvent *event);
    void keyReleaseEvent (QKeyEvent *event);

    GLRenderThread RenderThread;
    bool Ctrl_or_Meta_key_pressed;

private:

};

#endif // GLFRAME_H
