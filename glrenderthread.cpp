#include "/System/Library/Frameworks/GLUT.framework/Versions/A/Headers/glut.h"
#include "glrenderthread.h"
#include "glframe.h"
#include <math.h>

#include <iostream>
#include <QSound>
using namespace std;


GLRenderThread::GLRenderThread (GLFrame *_GLFrame) :
    QThread (),
    GLFrame_var (_GLFrame)
{
    doRendering = true;
    doResize = false;
    FrameCounter = 0;

    _zoom = -10;
    _current_mouse_pos = QPoint (0,0);

    // added 3 example points
    _point_cloud.push_back (20.0f);
    _point_cloud.push_back (0.0f);
    _point_cloud.push_back (-25.0f);

    _point_cloud.push_back (20.0f);
    _point_cloud.push_back (0.0f);
    _point_cloud.push_back (-15.0f);

    _point_cloud.push_back (20.0f);
    _point_cloud.push_back (0.0f);
    _point_cloud.push_back (-5.0f);
}

void GLRenderThread::resizeViewport (const QSize &size)
{
    w = size.width ();
    h = size.height ();
    doResize = true;
}

void GLRenderThread::stop()
{
    doRendering = false;
}


void GLRenderThread::run ()
{
    GLFrame_var->makeCurrent ();
    GLInit ();

    while (doRendering)
    {
        if (doResize)
        {
            GLResize (w, h);
            doResize = false;
        }
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        paintGL(); // render actual frame

        FrameCounter++;
        GLFrame_var->swapBuffers();

		msleep (16); // wait 16ms => about 60 FPS
    }
}


void GLRenderThread::GLInit (void)
{
    glClearColor (0.05f, 0.05f, 0.2f, 0.0f);

    glShadeModel (GL_SMOOTH); //set the shader to smooth shader
}


void GLRenderThread::GLResize (int width, int height)
{
    glViewport (0, 0, width, height);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    gluPerspective (45.,((GLfloat)width)/((GLfloat)height),0.1f,1000.0f);

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    gluLookAt (0, 0,-1,   // CameraPosition
               0, 0 ,0,   // CameraLookAt
               0, 1 ,0);  // CameraUp
}

void traceGrid ()
{
    // grid
    for (float i = -10; i <= 10; i += 1)
    {
        glBegin (GL_LINES);
            glColor3ub (75, 75, 75);
            glVertex3f (-10, 0, i);
            glVertex3f (10, 0, i);
            glVertex3f (i, 0,-10);
            glVertex3f (i, 0, 10);
        glEnd ();
    }

    // contour
    glBegin (GL_LINES);
    glColor3ub (45, 45, 45);
    glVertex3f (-20, -20, 0);
    glVertex3f (20, -20, 0);
    glVertex3f (20, -20, 0);
    glVertex3f (20, 20, 0);
    glVertex3f (20, 20, 0);
    glVertex3f (-20, 20, 0);
    glVertex3f (-20, 20, 0);
    glVertex3f (-20, -20, 0);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (45, 45, 45);
    glVertex3f (0, -20, -20);
    glVertex3f (0, 20, -20);
    glVertex3f (0, 20, -20);
    glVertex3f (0, 20, 20);
    glVertex3f (0, 20, 20);
    glVertex3f (0, -20, 20);
    glVertex3f (0, -20, 20);
    glVertex3f (0, -20, -20);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (45, 45, 45);
    glVertex3f (-20, 0, -20);
    glVertex3f (20, 0, -20);
    glVertex3f (20, 0, -20);
    glVertex3f (20, 0, 20);
    glVertex3f (20, 0, 20);
    glVertex3f (-20, 0, 20);
    glVertex3f (-20, 0, 20);
    glVertex3f (-20, 0, -20);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (45, 45, 45);
    glVertex3f (0, -20, -20);
    glVertex3f (0, 20, -20);
    glVertex3f (0, 20, -20);
    glVertex3f (0, 20, 20);
    glVertex3f (0, 20, 20);
    glVertex3f (0, -20, 20);
    glVertex3f (0, -20, 20);
    glVertex3f (0, -20, -20);
    glEnd ();

    // main axis
    glBegin (GL_LINES);
    glColor3ub (160, 0, 0);
    glVertex3f (0, 0, 0);
    glVertex3f (0, 0, 10);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 160, 0);
    glVertex3f (0, 0, 0);
    glVertex3f (0, 10, 0);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 0, 160);
    glVertex3f (0, 0, 0);
    glVertex3f (10, 0, 0);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (45, 45, 45);
    glVertex3f (10, 0, 0);
    glVertex3f (20, 0, 0);
    glVertex3f (0, 10, 0);
    glVertex3f (0, 20, 0);
    glVertex3f (0, 0, 10);
    glVertex3f (0, 0, 20);
    glEnd ();
}

void GLRenderThread::paintGL (void)
{
    _delta = QPoint (0,0);

    QPoint q;
    if (GLFrame_var->getCtrlMetaFlag()==true)
    {
        q = _mouse.pos();
        _delta = q - _last_pos;
        _last_pos = q;
    }
    else
    {
        q = _mouse.pos();
        _delta = QPoint (0,0);
        _last_pos = q;
    }

    _current_mouse_pos += _delta;

    //glShadeModel (GL_SMOOTH);
    glTranslatef (0.0f, 0.0f, -60.0f);  // move 60 units into the screen
    glRotatef (FrameCounter, 0.0f, _current_mouse_pos.x() * 0.01f, _current_mouse_pos.y() * 0.01f);

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Z−Buffer
    // Set the camera orientation :
    glMatrixMode (GL_MODELVIEW) ;
    glLoadIdentity ( ) ;
    gluLookAt (0,  0,-60,  // CameraPosition
               0,  0,  0,  // CameraLookAt
               0,  1,  0); // CameraUp

    // Rotate and zoom camera - Maya-like controls.
    glTranslatef (0, 0, -_zoom);
    // glTranslatef (tx,ty, 0);
    glTranslatef (0, 0, 0);
    glRotatef (-30 - _current_mouse_pos.y() * 0.15f,  1, 0, 0);
    glRotatef (-30 + _current_mouse_pos.x() * 0.1f,   0, 1, 0);

    traceGrid ();

    glEnable (GL_POINT_SMOOTH);
    glPointSize (4.0);

    uint j;
    uint len = _point_cloud.size()/3;
    for (uint i=0; i<len; i++)
    {
        j = i+i+i;
        glBegin (GL_POINTS);
            glVertex3f (_point_cloud[j], _point_cloud[j+1], _point_cloud[j+2]); glColor3f (1.,0.,0.);
        glEnd ();
    }
}

void GLRenderThread::updateZoom (float zoom)
{
    _zoom += zoom * 0.01;
}

void GLRenderThread::setPointCloud (vector<float> &point_cloud)
{
    // remove old test points.
    _point_cloud.pop_back();  _point_cloud.pop_back();  _point_cloud.pop_back();
    _point_cloud.pop_back();  _point_cloud.pop_back();  _point_cloud.pop_back();
    _point_cloud.pop_back();  _point_cloud.pop_back();  _point_cloud.pop_back();

    // add new point cloud
    for (vector <float>::iterator i=point_cloud.begin(); i!=point_cloud.end(); ++i)
    {
        _point_cloud.push_back(*i);
    }

    QSound ping ("/Users/macbookpro/git/JAPCE/ping.wav");
    ping.play();
}
