#include <QResizeEvent>
#include "glframe.h"
#include <iostream>
#include <vector>
using namespace std;


GLFrame::GLFrame (QWidget *parent) :
	QGLWidget (parent),
	RenderThread (this)
{
    setFocusPolicy (Qt::StrongFocus);
    setAutoBufferSwap (false);

    Ctrl_or_Meta_key_pressed = false;
}

GLFrame::~GLFrame() {}

void GLFrame::initRenderThread (void)
{
    doneCurrent();
    RenderThread.start();
}

void GLFrame::stopRenderThread (void)
{
    RenderThread.stop();
    RenderThread.wait();
}

void GLFrame::resizeEvent (QResizeEvent *event)
{
    RenderThread.resizeViewport (event->size());
}

void GLFrame::paintEvent (QPaintEvent *)
{
    // Do nothing. Let the thread do the work
}

void GLFrame::closeEvent (QCloseEvent *event)
{
    stopRenderThread();
    QGLWidget::closeEvent (event);
}

void GLFrame::wheelEvent (QWheelEvent *event)
{
    if (this->getCtrlMetaFlag() == true)
    {
        RenderThread.updateZoom (event->delta());
    }
}

void GLFrame::keyPressEvent (QKeyEvent *event)
{
    cout << "pressed" << endl;
    if (event->key() == Qt::Key_Control or event->key() == Qt::Key_Meta) {Ctrl_or_Meta_key_pressed = true;}
}

void GLFrame::keyReleaseEvent (QKeyEvent *event)
{
    cout << "released" << endl;
    if (event->key() == Qt::Key_Control or event->key() == Qt::Key_Meta) {Ctrl_or_Meta_key_pressed = false;}
}

bool GLFrame::getCtrlMetaFlag ()
{
    return Ctrl_or_Meta_key_pressed;
}

void GLFrame::displayPointCloud (vector<float> &point_cloud)
{    
    RenderThread.setPointCloud (point_cloud);
}
