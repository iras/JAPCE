#include "mainwindow.h"
#include "ui_mainwindow.h"

// pcl imports : they need to be here temporarily to avoid BOOST_JOIN error. That's down to an old QT bug that should've been fixed now. That also entails I can't extract the function doPCSegmentsRegistration into another class for the time being.

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/pca.h>
//#include <pcl/search/pcl_search.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/pcl_base.h>
//#include <pcl/console/print.h>
//#include <pcl/registration/boost.h>
//#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_rejection_distance.h>
//#include <pcl/registration/correspondence_rejection_trimmed.h>
//#include <pcl/registration/correspondence_rejection_one_to_one.h>

using namespace std;




MainWindow::MainWindow (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::MainWindow)
{
    ui->setupUi (this);

    _GLFrame = new GLFrame ();
    ui->verticalLayout->addWidget (_GLFrame);
    _GLFrame->initRenderThread();

    // setting the ending sound up.
    _ping = new QSound ("/Users/macbookpro/git/JAPCE/ping.wav");
    _ping->setLoops (1);

    // reset of labels' texts.
    ui->label_2->setText ("-");
    ui->label_3->setText ("-");
    ui->label_5->setText ("-");
    time_t result = time(NULL);
    ui->label_8->setText ("started approx at \n" + QString (std::asctime (std::localtime (&result))));

    _pc = new PointCloud ();

    _O = (cv::Mat_<double>(3,4) <<  1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0);

    _K = (cv::Mat_<double>(3,3) << 3117.75,     0.0,  1629.3,
                                      0.0,   3117.74, 1218.01,
                                      0.0,      0.0,     1.0);
}


MainWindow::~MainWindow ()
{
    delete _GLFrame;
    delete ui;
}

void MainWindow::closeEvent (QCloseEvent *evt)
{
    _GLFrame->stopRenderThread (); // stop the thread b4 exiting.
    QMainWindow::closeEvent (evt);
}

void MainWindow::on_pushButton_clicked ()
{
    QString filename = QFileDialog::getOpenFileName (this, tr ("Open Image"), "/Users/macbookpro/Dropbox/OpenCV2cookbook/test_photos", tr ("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    _image = cv::imread (filename.toAscii().data ());
    cv::cvtColor (_image, _image, CV_BGR2GRAY);

    // TODO : check that the images are all of the same size before pushing each back.

    _camera_shots.push_back (CameraShot (_image));

    if (_camera_shots.size()>1) {ui->pushButton_2->setEnabled (TRUE);}

    //cout << filename1.toAscii().data() << endl;
    //cv::namedWindow ("Original Image 1");
    //cv::imshow ("Original Image 1", image1);
}

void MainWindow::on_pushButton_3_clicked () {}

void MainWindow::on_pushButton_2_clicked ()
{
    // save the current time.
    _start = clock ();

    // set up feature detector.
    cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector (10);

    // set up robust matcher.
    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel (0.98);
    rmatcher.setMinDistanceToEpipolar (1.0);
    rmatcher.setRatio (0.65f);
    rmatcher.setFeatureDetector (pfd);

    // set up reconstructor and a few other details.
    Reconstructor rec;
    _GLFrame->delegateSetPointCloud (_pc);
    cv::Mat_<double> *P1, *P2;
    PCSegment *pcs, *previous_pcs;

    // loop thru to
    //      (1) instantiate pc segments.
    //      (2) fill out each pc segment with reconstructed 3D datasets, each one out of a pairs of camera shots.
    for (unsigned int n=0; n<_camera_shots.size()-1; n++)
    {
        _pc->addPCSegment (&(_camera_shots[n]), &(_camera_shots[n+1]));
        pcs = _pc->getPCSegment (n);
        previous_pcs = pcs;

        P1 = _camera_shots[n].getP ();
        P2 = _camera_shots[n+1].getP ();

        // match two camera shots
        cv::Mat f = this->getFundamentalAndMatches (rmatcher,
                                                    pcs->getMatches (),
                                                    _camera_shots[n].getImage(),   _camera_shots[n].getKeyPoints(),
                                                    _camera_shots[n+1].getImage(), _camera_shots[n+1].getKeyPoints());
        pcs->squeezePointsVectorsOutOfMatches ();

        this->displayMatches (f,
                              pcs->getMatches (),
                              pcs->getPoints1 (),
                              pcs->getPoints2 (),
                              _camera_shots[n].getImage(),   _camera_shots[n].getKeyPoints(),
                              _camera_shots[n+1].getImage(), _camera_shots[n+1].getKeyPoints());

        // calculate P2
        if (n > 0) {previous_pcs =_pc->getPCSegment (n-1);}
        *P1 = _O.clone ();
        *P2 = rec.calculateP2 (*P1,
                               rec.getPCandidatesfromFundamentalMtx ((cv::Mat_<double>) f),
                               *(pcs->getPoints1 ()),
                               *(pcs->getPoints2 ()));

        // do 3d reconstruction. (point cloud extraction and display)
        rec.doTriangulationSweep (P1, P2,
                                  pcs->getPoints1 (),
                                  pcs->getPoints2 (),
                                  pcs->getVectorX ());

        _GLFrame->DelegateCameraPyramidAddition (*P1, _image.rows, _image.cols);
        _GLFrame->DelegateCameraPyramidAddition (*P2, _image.rows, _image.cols);
    }

    this->doPCSegmentsRegistration (_pc);





    // display the execution time
    _stop = clock ();
    ui->label_8->setText (QString::number ((_stop - _start)/(CLOCKS_PER_SEC * 60)) + " minutes");

    // display no. of tracked points
    //ui->label_2->setText    ("<b>" + QString::number (points1.size()) + "</b>");
    //ui->label_test->setText ("<b>" + QString::number (rec.getIndex()) + "</b>");

    _ping->play ();

}



void MainWindow::doPCSegmentsRegistration (PointCloud *pc)
{
    cv::Point3d t; // temp var
    vector<cv::DMatch> *matches1, *matches2;
    vector<cv::Point3d> *X1, *X2;
    vector<cv::Point3f> Xm1, Xm2;

    for (unsigned int n=0; n < pc->size()-1; n++)
    {
        // extract the 2 vectors of points, the 2nd of which will have to align to the 1st one using the estimated transform below.
        matches1 = pc->getPCSegment (n)->getMatches ();
        matches2 = pc->getPCSegment (n+1)->getMatches ();
        X1 = pc->getPCSegment (n)->getVectorX ();
        X2 = pc->getPCSegment (n+1)->getVectorX ();

        Xm1.clear();
        Xm2.clear();

        for (unsigned int i=0; i < matches1->size (); i++)
        {
            int tmp1 = (*matches1)[i].trainIdx;
            for (unsigned int j=i; j < matches2->size (); j++)
            {
                int tmp2 = (*matches2)[j].queryIdx;
                if (tmp1 == tmp2)
                {
                    t = (*X1)[i];  Xm1.push_back (cv::Point3f (float(t.x), float(t.y), float(t.z)));
                    t = (*X2)[j];  Xm2.push_back (cv::Point3f (float(t.x), float(t.y), float(t.z)));

                    break;
                }
            }
        }

        cout << "\nNumber of matches " << Xm1.size() << "\n\n";

        // init point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr XPm1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr XPm2 (new pcl::PointCloud<pcl::PointXYZ>);
        // populate point clouds
        XPm1->width = Xm1.size ();   XPm1->height = 1;   XPm1->is_dense = false;   XPm1->points.resize (XPm1->width * XPm1->height);
        XPm2->width = Xm1.size ();   XPm2->height = 1;   XPm2->is_dense = false;   XPm2->points.resize (XPm2->width * XPm2->height);
        // turn the vectors Xm1 and Xm2 in PCL's specific pointcloud data.
        for (unsigned int i=0; i < Xm1.size(); i++)
        {
            t = Xm1[i];   XPm1->points[i].x = t.x;   XPm1->points[i].y = t.y;   XPm1->points[i].z = t.z;
            t = Xm2[i];   XPm2->points[i].x = t.x;   XPm2->points[i].y = t.y;   XPm2->points[i].z = t.z;
        }

        pcl::registration::TransformationEstimationSVDScale <pcl::PointXYZ, pcl::PointXYZ> te;
        Eigen::Matrix4f T;
        //pcl::CorrespondencesPtr corrs (new pcl::Correspondences());
        te.estimateRigidTransformation (*XPm2, *XPm1, T);
        //pcl::transformPointCloud (*source_segmented_, *source_transformed_, T);

        // convert the Eigen matrix to cv:Mat_.
        cv::Mat_<float> Tcv = (cv::Mat_<float>(4, 4));
        for (int m = 0; m < 16; m++)   Tcv.at<float> (m/4, m%4) = T(m/4, m%4);


        cout << T << "\n\n";

        //cv::Mat registration_transform = this->doPCSegmentsRegistration (&Xm1, &Xm2);


        cv::Mat_<float> v;
        cv::Mat_<float> result;
        for (unsigned int i=0; i < X2->size(); i++)
        {
            t = (*X2)[i];
            v = (cv::Mat_<float>(4,1) << float(t.x), float(t.y), float(t.z), 1);

            result = Tcv * v;

            float* res = result.ptr<float>(0);
            (*X2)[i] = cv::Point3d (double (res[0]), double (res[1]), double (res[2]));
        }
    }
}



cv::Mat MainWindow::getFundamentalAndMatches (RobustMatcher rmatcher,
                                              vector<cv::DMatch> *matches,
                                              cv::Mat *img1,  vector<cv::KeyPoint> *keypoints1,
                                              cv::Mat *img2,  vector<cv::KeyPoint> *keypoints2)
{
    rmatcher.detectFeatures (*img1, *img2, *keypoints1, *keypoints2);
    ui->label_3->setText (QString::number (rmatcher.getNumberFeaturesImage1 ()));
    ui->label_5->setText (QString::number (rmatcher.getNumberFeaturesImage2 ()));
    cv::Mat f = rmatcher.match (*img1, *img2, *matches, *keypoints1, *keypoints2);

    return f;
}


void MainWindow::displayMatches (cv::Mat f,
                                 vector<cv::DMatch>  *matches,
                                 vector<cv::Point2f> *points1,
                                 vector<cv::Point2f> *points2,
                                 cv::Mat *img1,  vector<cv::KeyPoint> *keypoints1,
                                 cv::Mat *img2,  vector<cv::KeyPoint> *keypoints2)
{
    // plot the matches

    // clone images and change the image format to BGR otherwise we can't see points in red.
    cv::Mat img1_clone = img1->clone ();
    cv::Mat img2_clone = img2->clone ();
    cv::cvtColor (img1_clone, img1_clone, CV_GRAY2BGR);
    cv::cvtColor (img2_clone, img2_clone, CV_GRAY2BGR);

    /*
     cv::Mat imageMatches;
    // place Qt image.
    //                 1st image + keypoints     2nd image + keypoints      matches   achieved image   lines colour
    //cv::drawMatches (img1_clone, *keypoints1,  img2_clone, *keypoints2,  *matches,  imageMatches,    cv::Scalar (0,0,255));
    QImage img = QImage ((uchar*)imageMatches.data, imageMatches.cols, imageMatches.rows, imageMatches.step, QImage::Format_RGB888 );
    ui->label->setPixmap (QPixmap::fromImage (img)); // display on label
    ui->label->resize (ui->label->pixmap()->size()); // resize the label to fit the image
    */

    // red circle the matches.
    for (unsigned int i = 0; i<points1->size(); i++)
        {
            cv::Point2f p1 = points1->at(i);
            cv::circle (img1_clone, p1, 4, cv::Scalar(0,0,255), 3); // B G R

            cv::Point2f p2 = points2->at(i);
            cv::circle (img2_clone, p2, 4, cv::Scalar(0,0,255), 3); // B G R
        }

    // draw epipolar lines
    // 1st image
    vector<cv::Vec3f> lines1;
    cv::computeCorrespondEpilines (cv::Mat (*points1), 1, f, lines1);
    for (vector<cv::Vec3f>::const_iterator it = lines1.begin(); it!=lines1.end(); ++it)
    {
        cv::line  (img2_clone, cv::Point (0,-(*it)[2]/(*it)[1]),
        cv::Point (img2_clone.cols, -((*it)[2]+(*it)[0] * img2_clone.cols)/(*it)[1]),  cv::Scalar (100,160,100));
    }
    // 2nd image
    vector<cv::Vec3f> lines2;
    cv::computeCorrespondEpilines (cv::Mat (*points2), 2, f, lines2);
    for (vector<cv::Vec3f>::const_iterator it = lines2.begin(); it!=lines2.end(); ++it)
    {
        cv::line  (img1_clone, cv::Point(0,-(*it)[2]/(*it)[1]),
        cv::Point (img1_clone.cols, -((*it)[2]+(*it)[0] * img1_clone.cols)/(*it)[1]),  cv::Scalar (100,150,100));
    }

    // display images with epipolar lines separately from the main window.
    //cv::namedWindow ("Right Image - Epilines (RANSAC)");
    cv::imshow ("Right Image Epilines (RANSAC)", img1_clone);
    //cv::namedWindow ("Left Image - Epilines (RANSAC)");
    cv::imshow ("Left Image Epilines (RANSAC)",  img2_clone);
}
