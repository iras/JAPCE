#include "mainwindow.h"
#include "ui_mainwindow.h"
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

    _O = (cv::Mat_<double>(3,4) <<  1,0,0,0,  0,1,0,0,  0,0,1,0);
    //_B = _O;


    test ();
}

void MainWindow::test ()
{
    vector<cv::Point3f> first, second;
    vector<uchar> inliers;
    cv::Mat aff(3,4,CV_64F);

    for (int i = 0; i <6; i++)
    {
        first.push_back(cv::Point3f(i,i%3,1));
        second.push_back(cv::Point3f(i,i%3,1));
    }

    int ret = cv::estimateAffine3D(first, second, aff, inliers);
    cout << aff << endl;
    cout << endl;
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
    // take the time.
    _start = clock();

    // feature detector
    cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector (10);

    // set up the robust matcher.
    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel (0.98);
    rmatcher.setMinDistanceToEpipolar (1.0);
    rmatcher.setRatio (0.65f);
    rmatcher.setFeatureDetector (pfd);

    int image_rows = _image.rows;
    int image_cols = _image.cols;
    Reconstructor rec; // init the reconstructor
    _GLFrame->delegateSetPointCloud (_pc);

    for (unsigned int n=0; n<_camera_shots.size()-1; n++)
    {
        _pc->addPCSegment (&(_camera_shots[n]), &(_camera_shots[n+1]));
        PCSegment *pcs = _pc->getPCSegment (n);

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

        // do 3d reconstruction. (point cloud extraction and display)
        this->doReconstructionSweep (&rec,
                                     f,
                                     image_rows, image_cols,
                                     pcs->getPoints1 (),
                                     pcs->getPoints2 (),
                                     pcs->getVectorX ());
    }

    // display the execution time
    _stop = clock ();
    ui->label_8->setText (QString::number ((_stop - _start)/(CLOCKS_PER_SEC * 60)) + " minutes");

    // display no. of tracked points
    //ui->label_2->setText    ("<b>" + QString::number (points1.size()) + "</b>");
    //ui->label_test->setText ("<b>" + QString::number (rec.getIndex()) + "</b>");

    _ping->play ();
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


void MainWindow::doReconstructionSweep (Reconstructor *rec,
                                        cv::Mat f,
                                        int image_rows,  int image_cols,
                                        vector<cv::Point2f> *points1,
                                        vector<cv::Point2f> *points2,
                                        vector<cv::Point3d> *pc_segment)
{
    vector<cv::Point3d> *pc_segment_ref = pc_segment;

    cv::Mat_<double> P1 = _O;
    cv::Mat_<double> P2 = rec->pickTheRightP (P1, rec->getPCandidatesfromFundamentalMtx ((cv::Mat_<double>)f), *points1, *points2);
    //this->adjustMatrixToLatestOrigin (&P1, &_B); // this is redundant as P1 oould've been set straight to the value in _B.
    //this->adjustMatrixToLatestOrigin (&P2, &_B);
    //_B = P2.clone (); // set to P2 so it's ready for the next sweep. TODO : This assignment needs to be taken out of the function doReconstructionSweep (...) otherwise it's confusing.

    cv::Mat_<double> x1, x2;
    cv::Mat_<double> X; // 3D coordinates for a reconstructed point.
    double* p;
    cv::Point3d point;

    for (unsigned int i=0; i < points1->size(); i++)
    {
        x1 = (cv::Mat_<double>(3,1) <<  (points1->at(i)).x, (points1->at(i)).y, 1.0);
        x2 = (cv::Mat_<double>(3,1) <<  (points2->at(i)).x, (points2->at(i)).y, 1.0);

        X = rec->triangulate (x1, x2, P1, P2);

        p = X.ptr<double>(0);
        point = cv::Point3d (p[0], p[1], p[2]);
        pc_segment_ref->push_back (point); // push the vector point into the vector pc_segment.
    }

    _GLFrame->DelegateCameraPyramidAddition (P1, image_rows, image_cols);
    _GLFrame->DelegateCameraPyramidAddition (P2, image_rows, image_cols);
}


void MainWindow::doPCSegmentsRegistration (void) // align 2 point cloud segments
{
}

void MainWindow::adjustMatrixToLatestOrigin (cv::Mat *P, cv::Mat *B)  // TODO : remove this function, the solvePnP will replace this function.
{
    // the camera matrix is a 3x4 matrix [R|t]where R is the rotation matrix and t is the translation vector.
    // Both *P and *B are like the above's matrix. The adjustment consists of changing *P's basis.

    cout << *P << endl;

    cv::Mat_<double> cols = (cv::Mat_<double>(3,1) <<  P->at<double>(0,3) + B->at<double>(0,3), P->at<double>(1,3) + B->at<double>(1,3), P->at<double>(2,3) + B->at<double>(2,3)); // cols contains the sum of the base's and camera matrix's translations.

    cv::Mat_<double> B_Rot = B->colRange(0,3).rowRange(0,3); // 3x3 rotation matrix
    cv::Mat_<double> P_Rot = P->colRange(0,3).rowRange(0,3); // 3x3 rotation matrix

    cv::Mat_<double> intermediate_matrix = B_Rot * P_Rot; //

    cv::hconcat (intermediate_matrix, cols, *P); // add the cols to the intermediate matrix and whack it into the camera matrix.

    cout << *P << "\n\n";
}
