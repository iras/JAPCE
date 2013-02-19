#include "glrenderthread.h"
#include "glframe.h"
#include <math.h>

#include <iostream>
using namespace std;


GLRenderThread::GLRenderThread (GLFrame *_GLFrame) :
    QThread (),
    GLFrame_var (_GLFrame)
{
    doRendering = true;
    doResize = false;
    FrameCounter = 0;

    _camera_dist = -10;
    _o = QVector3D (0.0, 0.0, 0.0); // current origin
    _current_angles = QPoint (0,0);

    // init m with 4x4 identity matrix
    for (int i=0; i<16; i++) {_m[i] = 0;}
    _m[0]=1; _m[5]=1; _m[10]=1; _m[15]=1;

    // added loader's animation
    this->addLoaderAnim ();

    _point_cloud = new PointCloud(); // create an empty PointCloud instance so that the render thread can render an empty point cloud, this instance will be replaced as soon as the main point cloud has at least one segment to render.
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

void GLRenderThread::drawGrid (void)
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

void GLRenderThread::drawCurrentOriginAxes (void)
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

void GLRenderThread::addCameraPyramid  (cv::Mat_<double> &camera_matrix, int rows, int cols)
{
    double rh = rows / 1500;
    double ch = cols / 1500;
    double depth = 3.0;
    cv::Mat_<double> pp0, pp1, pp2, pp3, pp4;

    cv::Mat_<double> p0 = (cv::Mat_<double>(4,1) <<  0.0, 0.0,  0.0,  1.0);
    cv::Mat_<double> p1 = (cv::Mat_<double>(4,1) <<  -ch, -rh, depth, 1.0);
    cv::Mat_<double> p2 = (cv::Mat_<double>(4,1) <<  -ch,  rh, depth, 1.0);
    cv::Mat_<double> p3 = (cv::Mat_<double>(4,1) <<   ch,  rh, depth, 1.0);
    cv::Mat_<double> p4 = (cv::Mat_<double>(4,1) <<   ch, -rh, depth, 1.0);

    pp0 = camera_matrix * p0;
    pp1 = camera_matrix * p1;
    pp2 = camera_matrix * p2;
    pp3 = camera_matrix * p3;
    pp4 = camera_matrix * p4;

    vector<cv::Mat_<double> > camera_pyramid;
    camera_pyramid.push_back (pp0);
    camera_pyramid.push_back (pp1);
    camera_pyramid.push_back (pp2);
    camera_pyramid.push_back (pp3);
    camera_pyramid.push_back (pp4);

    camera_pyramids.push_back (camera_pyramid);
}

void GLRenderThread::displayCameraPyramids  (void)
{
    unsigned int len = camera_pyramids.size();
    for (unsigned int i=0; i<len; i++)
    {
        vector <cv::Mat_<double> > cp;
        cp = camera_pyramids[i];

        // camera pyramid
        glBegin (GL_LINES);
        glColor3ub (150, 150, 0);
        glVertex3f (cp[0](0), cp[0](1), cp[0](2));
        glVertex3f (cp[1](0), cp[1](1), cp[1](2));

        glVertex3f (cp[1](0), cp[1](1), cp[1](2));
        glVertex3f (cp[2](0), cp[2](1), cp[2](2));

        glVertex3f (cp[2](0), cp[2](1), cp[2](2));
        glVertex3f (cp[3](0), cp[3](1), cp[3](2));

        glVertex3f (cp[3](0), cp[3](1), cp[3](2));
        glVertex3f (cp[4](0), cp[4](1), cp[4](2));

        glVertex3f (cp[4](0), cp[4](1), cp[4](2));
        glVertex3f (cp[1](0), cp[1](1), cp[1](2));

        glVertex3f (cp[0](0), cp[0](1), cp[0](2));
        glVertex3f (cp[2](0), cp[2](1), cp[2](2));

        glVertex3f (cp[0](0), cp[0](1), cp[0](2));
        glVertex3f (cp[3](0), cp[3](1), cp[3](2));

        glVertex3f (cp[0](0), cp[0](1), cp[0](2));
        glVertex3f (cp[4](0), cp[4](1), cp[4](2));
        glEnd ();
    }
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

            GLfloat vec_x = - _m[0]*delta_x + _m[1]*delta_y + _m[2]*0 ;
            GLfloat vec_y = - _m[4]*delta_x + _m[5]*delta_y + _m[6]*0 ;
            GLfloat vec_z = - _m[8]*delta_x + _m[9]*delta_y + _m[10]*0 ;

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
    //               CameraPos                  CameraLookAt          CameraUp
    gluLookAt (_o.x(),_o.y(),_o.z()-40,     _o.x(),_o.y(),_o.z(),     0, 1, 0);

    // Rotate and move camera forward or backward (Maya-like 3D navigation).
    glTranslatef (0, 0, -_camera_dist);
    glTranslatef (_o.x(), _o.y(), _o.z()); // rotation around current origin.          Part 1/3
    glRotatef (-30 - _current_angles.y() * 0.15f,  1, 0, 0); // rotate around axis x.  Part 2/3
    glRotatef (-30 + _current_angles.x() * 0.1f,   0, 1, 0); // rotate around axis y.  Part 2/3
    glTranslatef (-_o.x(), -_o.y(), -_o.z()); // rotation around current origin.       Part 3/3

    // get the camera matrix (or anyway the main matrix) and put it into the GLfloat m[16].
    glGetFloatv (GL_MODELVIEW_MATRIX, _m);

    // dispaly grid and current origin.
    this->drawGrid ();
    this->drawCurrentOriginAxes ();
    this->displayCameraPyramids ();


    vector <float> temp;
    temp.resize (3);

    // display the loader anim
    for (unsigned int i=0; i<_loader_anim.size(); i++)
    {
        glBegin (GL_POINTS);
        temp.clear();
        temp = vector<float> (_loader_anim[i]);
        glVertex3f (temp[0], temp[1], temp[2]); glColor3f (1., 0., 0.);
        glEnd ();
    }



    // display the point cloud
    glEnable (GL_POINT_SMOOTH);
    glPointSize (4.0);
    vector<cv::Point3d> *meta_temp;
    for (unsigned int i=0; i<_point_cloud->size(); i++)
    {
        PCSegment *pcs = _point_cloud->getPCSegment (i);
        meta_temp = pcs->getVectorX ();

        for (unsigned int j=0; j<meta_temp->size(); j++)
        {
            glBegin (GL_POINTS);

            temp.clear();
            cv::Point3d *temp ;
            temp = &(meta_temp->at(j));
            glVertex3f (temp->x, temp->y, temp->z); glColor3f (1., 1. - float(i), 0.);
            glEnd ();
        }
    }
}

void GLRenderThread::updateCameraDistanceFromCenter (float dist)
{
    _camera_dist += dist * 0.01;
}

void GLRenderThread::setPointCloud (PointCloud *point_cloud)
{
    _point_cloud = point_cloud;
}


////////// additional functions


void GLRenderThread::addLoaderAnim (void)
{
    // add three points
    vector<float> temp;
    temp.resize (3); // fix temp size to 3 floats.
    vector<vector<float> > meta_temp;
    meta_temp.clear ();

    temp.clear ();
    temp.push_back (20.0f);
    temp.push_back (0.0f);
    temp.push_back (-25.0f);
    _loader_anim.push_back (temp);

    temp.clear ();
    temp.push_back (20.0f);
    temp.push_back (0.0f);
    temp.push_back (-15.0f);
    _loader_anim.push_back (temp);

    temp.clear ();
    temp.push_back (20.0f);
    temp.push_back (0.0f);
    temp.push_back (-5.0f);
    _loader_anim.push_back (temp);
}

void GLRenderThread::removeLoaderAnim (void)
{
    // remove old test points.
    _loader_anim.clear ();
}
