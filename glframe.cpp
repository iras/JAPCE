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
    Alt_key_pressed = false;
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

void GLFrame::setPointCloud (vector<vector<vector<float> > > point_cloud)
{
    RenderThread.setPointCloud (point_cloud);
}

void GLFrame::DelegateCameraPyramidAddition  (cv::Mat_<double> &camera_matrix, int rows, int cols)
{
    RenderThread.addCameraPyramid (camera_matrix, rows, cols);
}

////////////////

void GLFrame::resizeEvent (QResizeEvent *event)
{
    RenderThread.resizeViewport (event->size());
}

void GLFrame::paintEvent (QPaintEvent *)
{
    // Do nothing. Let the thread do the heavy lifting.
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
        RenderThread.updateCameraDistanceFromCenter (event->delta());
    }
}

void GLFrame::keyPressEvent (QKeyEvent *event)
{
    if (event->key() == Qt::Key_Control or event->key() == Qt::Key_Meta) {Ctrl_or_Meta_key_pressed = true;}
    if (event->key() == Qt::Key_Alt) {Alt_key_pressed = true;}
}

void GLFrame::keyReleaseEvent (QKeyEvent *event)
{
    if (event->key() == Qt::Key_Control or event->key() == Qt::Key_Meta) {Ctrl_or_Meta_key_pressed = false;}
    if (event->key() == Qt::Key_Alt) {Alt_key_pressed = false;}
}

bool GLFrame::getCtrlMetaFlag () {return Ctrl_or_Meta_key_pressed;}
bool GLFrame::getAltFlag ()      {return Alt_key_pressed;}
