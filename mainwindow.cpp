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

    _pc = new MPointCloud ();
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

/////////////////////////////////////////////////////////////////

void MainWindow::on_pushButton_clicked ()
{
    QString filename1 = QFileDialog::getOpenFileName (this, tr ("Open Image"), "/Users/macbookpro/Dropbox/OpenCV2cookbook/test_photos", tr ("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    _image1 = cv::imread (filename1.toAscii().data ());
    cv::cvtColor (_image1, _image1, CV_BGR2GRAY);

    //cout << filename1.toAscii().data() << endl;
    //cv::namedWindow ("Original Image 1");
    //cv::imshow ("Original Image 1", image1);
}

void MainWindow::on_pushButton_3_clicked()
{
    QString filename2 = QFileDialog::getOpenFileName (this, tr ("Open Image"), "/Users/macbookpro/Dropbox/OpenCV2cookbook/test_photos", tr ("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    _image2 = cv::imread(filename2.toAscii().data ());
    cv::cvtColor (_image2, _image2, CV_BGR2GRAY);

    //cv::namedWindow ("Original Image 2");
    //cv::imshow ("Original Image 2", image2);
}


void MainWindow::on_pushButton_2_clicked ()
{
    // take the time
    _start = clock();

    // set up the robust matcher.
    RobustMatcher rmatcher;
    rmatcher.setConfidenceLevel (0.98);
    rmatcher.setMinDistanceToEpipolar (1.0);
    rmatcher.setRatio (0.65f);
    cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(10);
    rmatcher.setFeatureDetector (pfd);

    int image_rows = _image1.rows;
    int image_cols = _image1.cols;
    vector<cv::Point2f> points1;
    vector<cv::Point2f> points2;
    _GLFrame->delegateSetPointCloud (_pc);
    Reconstructor rec; // set the reconstructor up.

    // match two images
    cv::Mat f = getFundamentalAndMatches (&points1, &points2, rmatcher);

    // 3d reconstruction. (point cloud extraction and display)
    doReconstructSweep (&rec, f, image_rows, image_cols, &points1, &points2);



    // display the execution time
    _stop = clock ();
    ui->label_8->setText (QString::number ((_stop - _start)/(CLOCKS_PER_SEC * 60)) + " minutes");

    // display no. of tracked points
    ui->label_2->setText    ("<b>" + QString::number (points1.size()) + "</b>");
    ui->label_test->setText ("<b>" + QString::number (rec.getIndex()) + "</b>");

    _ping->play ();
}

cv::Mat MainWindow::getFundamentalAndMatches (vector<cv::Point2f> *points1, vector<cv::Point2f> *points2, RobustMatcher rmatcher)
{
    vector<cv::DMatch> matches;
    vector<cv::KeyPoint> keypoints1, keypoints2;
    rmatcher.detectFeatures (_image1, _image2, keypoints1, keypoints2);
    ui->label_3->setText (QString::number (rmatcher.getNumberFeaturesImage1 ()));
    ui->label_5->setText (QString::number (rmatcher.getNumberFeaturesImage2 ()));
    cv::Mat f = rmatcher.match (_image1, _image2, matches, keypoints1, keypoints2);

    // plot the matches
    cv::Mat imageMatches;
    // change the images format to BGR otherwise we can't see points in red.
    cv::cvtColor (_image1, _image1, CV_GRAY2BGR);
    cv::cvtColor (_image2, _image2, CV_GRAY2BGR);
    //               1st image + keypoints  2nd image + keypoints  matches   achieved image   lines colour
    cv::drawMatches (_image1, keypoints1,    _image2, keypoints2,    matches,  imageMatches,    cv::Scalar (200,200,0));
    /*
    // place Qt image.
    QImage img = QImage ((uchar*)imageMatches.data, imageMatches.cols, imageMatches.rows, imageMatches.step, QImage::Format_RGB888 );
    ui->label->setPixmap (QPixmap::fromImage (img)); // display on label
    ui->label->resize (ui->label->pixmap()->size()); // resize the label to fit the image
    */

    // get Point2f vector from keypoints vector
    for (vector<cv::DMatch>::const_iterator it=matches.begin(); it!=matches.end(); ++it)
    {
        float x1 = keypoints1 [it->queryIdx].pt.x;
        float y1 = keypoints1 [it->queryIdx].pt.y;
        points1->push_back (cv::Point2f(x1,y1));
        cv::circle (_image1, cv::Point (x1,y1), 4, cv::Scalar(0,0,255), 3); // B G R

        float x2 = keypoints2 [it->trainIdx].pt.x;
        float y2 = keypoints2 [it->trainIdx].pt.y;
        points2->push_back (cv::Point2f(x2,y2));
        cv::circle (_image2, cv::Point (x2,y2), 4, cv::Scalar(0,0,255), 3); // B G R
    }

    // draw epipolar lines
    // 1st image
    vector<cv::Vec3f> lines1;
    cv::computeCorrespondEpilines (cv::Mat(*points1), 1, f, lines1);
    for (vector<cv::Vec3f>::const_iterator it= lines1.begin(); it!=lines1.end(); ++it)
    {
        cv::line  (_image2,cv::Point (0,-(*it)[2]/(*it)[1]),
        cv::Point (_image2.cols,-((*it)[2]+(*it)[0]*_image2.cols)/(*it)[1]),
        cv::Scalar (100,160,100));
    }
    // 2nd image
    vector<cv::Vec3f> lines2;
    cv::computeCorrespondEpilines (cv::Mat (*points2), 2, f, lines2);
    for (vector<cv::Vec3f>::const_iterator it= lines2.begin(); it!=lines2.end(); ++it)
    {
        cv::line  (_image1,cv::Point(0,-(*it)[2]/(*it)[1]),
        cv::Point (_image1.cols,-((*it)[2]+(*it)[0]*_image1.cols)/(*it)[1]),
        cv::Scalar (100,150,100));
    }

    // display images with epipolar lines separately from the main window.
    //cv::namedWindow ("Right Image - Epilines (RANSAC)");
    cv::imshow ("Right Image Epilines (RANSAC)", _image1);
    //cv::namedWindow ("Left Image - Epilines (RANSAC)");
    cv::imshow ("Left Image Epilines (RANSAC)",  _image2);

    return f;
}

void MainWindow::doReconstructSweep (Reconstructor *rec, cv::Mat f, int image_rows, int image_cols, vector<cv::Point2f> *points1, vector<cv::Point2f> *points2)
{
    vector<vector<float> > pc_segment;

    cv::Mat_<double> P1 = (cv::Mat_<double>(3,4) <<  1,0,0,0,  0,1,0,0,  0,0,1,0);
    cv::Mat_<double> P2 = rec->pickTheRightP (P1, rec->getPCandidatesfromFundamentalMtx ((cv::Mat_<double>)f), *points1, *points2);
    cv::Mat_<double> x1, x2;
    cv::Mat_<double> tmp;
    double* tmpp;
    vector<float> temp;
    temp.resize (3); // fix temp size to 3 floats.

    for (uint i=0; i < points1->size(); i++)
    {
        x1 = (cv::Mat_<double>(3,1) <<  (points1->at(i)).x, (points1->at(i)).y, 1.0);
        x2 = (cv::Mat_<double>(3,1) <<  (points2->at(i)).x, (points2->at(i)).y, 1.0);

        tmp = rec->triangulate (x1, x2, P1, P2);

        tmpp = tmp.ptr<double>(0);
        temp.clear();
        temp.push_back (float (tmpp[0])); // create vector of coordinates
        temp.push_back (float (tmpp[1]));
        temp.push_back (float (tmpp[2]));
        pc_segment.push_back (temp); // push the vector temp into the point_cloud vector.
    }

    _pc->addSegment (&pc_segment);

    _GLFrame->DelegateCameraPyramidAddition (P1, image_rows, image_cols);
    _GLFrame->DelegateCameraPyramidAddition (P2, image_rows, image_cols);
}
