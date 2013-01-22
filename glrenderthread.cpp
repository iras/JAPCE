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
    _o = QVector3D (0.0, 0.0, 0.0); // current origin
    _current_angles = QPoint (0,0);

    // init m with 4x4 identity matrix
    for (int i=0; i<16; i++) {m[i] = 0;}
    m[0]=1; m[5]=1; m[10]=1; m[15]=1;

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

void GLRenderThread::traceGrid (void)
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
    glColor3ub (250, 0, 0);
    glVertex3f (0, 0, 0);
    glVertex3f (0, 0, 5);
    glColor3ub (125, 0, 0);
    glVertex3f (0, 0, 5);
    glVertex3f (0, 0, 20);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 250, 0);
    glVertex3f (0, 0, 0);
    glVertex3f (0, 5, 0);
    glColor3ub (0, 125, 0);
    glVertex3f (0, 5, 0);
    glVertex3f (0, 20, 0);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 0, 250);
    glVertex3f (0, 0, 0);
    glVertex3f (5, 0, 0);
    glColor3ub (0, 0, 125);
    glVertex3f (5, 0, 0);
    glVertex3f (20, 0, 0);
    glEnd ();
}

void GLRenderThread::traceCurrentOrigin (void)
{
    glPointSize (6.0);
    glBegin (GL_POINTS);
        glColor3f (0.0,1.0,1.0); glVertex3f (_o.x(), _o.y(), _o.z());
    glEnd ();

    // main axis
    glBegin (GL_LINES);
    glColor3ub (250, 0, 0);
    glVertex3f (_o.x(), _o.y(), _o.z());
    glVertex3f (_o.x(), _o.y(), _o.z()+5);
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 250, 0);
    glVertex3f (_o.x(), _o.y(), _o.z());
    glVertex3f (_o.x(), _o.y()+5, _o.z());
    glEnd ();
    glBegin (GL_LINES);
    glColor3ub (0, 0, 250);
    glVertex3f (_o.x(), _o.y(), _o.z());
    glVertex3f (_o.x()+5, _o.y(), _o.z());
    glEnd ();
}

void GLRenderThread::paintGL (void)
{
    _delta = QPoint (0,0);

    QPoint q;
    if (GLFrame_var->getCtrlMetaFlag()==true ^ GLFrame_var->getAltFlag()==true) // xor
    {
        if (GLFrame_var->getCtrlMetaFlag()==true) // rotation
        {
            q = _mouse.pos();
            _delta = q - _last_pos;
            _last_pos = q;
        }
        else // translation
        {
            q = _mouse.pos();

            // get the mouse deltas and transform them with the camera's local axes so that
            // it'll look like as if the camera will translate based on the local axes x and y.
            // What happens in reality is that the origin will shift.
            // Somehow similar to the Maya Camera Track Tool.
            GLfloat delta_x = q.x() - _last_pos.x();
            GLfloat delta_y = q.y() - _last_pos.y();

            GLfloat vec_x = - m[0]*delta_x + m[1]*delta_y + m[2]*0 ;
            GLfloat vec_y = - m[4]*delta_x + m[5]*delta_y + m[6]*0 ;
            GLfloat vec_z = - m[8]*delta_x + m[9]*delta_y + m[10]*0 ;

            _o.setX (_o.x() + 0.03 * vec_x);
            _o.setY (_o.y() + 0.03 * vec_y);
            _o.setZ (_o.z() + 0.03 * vec_z);

            _last_pos = q;
        }
    }
    else
    {
        q = _mouse.pos();
        _last_pos = q;
    }

    _current_angles += _delta;

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Z−Buffer

    // Set the camera orientation :
    glMatrixMode (GL_MODELVIEW) ;
    glLoadIdentity ();
    //         CameraPos         CameraLookAt          CameraUp
    gluLookAt (0, 0,-40,     _o.x(),_o.y(),_o.z(),     0, 1, 0);

    // Rotate and move camera forward or backward (Maya-like 3D navigation).
    glTranslatef (0, 0, -_zoom);
    glTranslatef (_o.x(), _o.y(), _o.z()); // rotation around current origin.          Part 1/3
    glRotatef (-30 - _current_angles.y() * 0.15f,  1, 0, 0); // rotate around axis x.  Part 2/3
    glRotatef (-30 + _current_angles.x() * 0.1f,   0, 1, 0); // rotate around axis y.  Part 2/3
    glTranslatef (-_o.x(), -_o.y(), -_o.z()); // rotation around current origin.       Part 3/3

    // get the camera matrix (or anyway the main matrix) and put it into the GLfloat m[16].
    glGetFloatv (GL_MODELVIEW_MATRIX, m);

    // dispaly grid and current origin.
    this->traceGrid ();
    this->traceCurrentOrigin ();

    // display the point cloud
    glEnable (GL_POINT_SMOOTH);
    glPointSize (4.0);
    unsigned int j;
    unsigned int len = _point_cloud.size()/3;
    for (unsigned int i=0; i<len; i++)
    {
        j = i+i+i;
        glBegin (GL_POINTS);
            glVertex3f (_point_cloud[j], _point_cloud[j+1], _point_cloud[j+2]); glColor3f (1.,0.,0.);
        glEnd ();
    }
}

void GLRenderThread::updateCameraDistanceFromCenter (float zoom)
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
