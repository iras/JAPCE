#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <vector>
#include <QtGui/QMainWindow>
#include <QtGui/QFileDialog>
#include <QSound>
#include <time.h>

#include "matcher.h"
#include "reconstructor.h"
#include "glframe.h"
#include <mpointcloud.h>

// usual OpenCV imports

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// NEW IMPORTS in OpenCV 2.4.3

#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/photo/photo.hpp"
#include <opencv2/legacy/legacy.hpp>

class GLFrame;

namespace Ui
{
    class MainWindow;
}



class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow (QWidget *parent = 0);
    ~MainWindow ();
    
    cv::Mat getFundamentalAndMatches (vector<cv::Point2f> *points1, cv::Mat *img1, vector<cv::Point2f> *points2, cv::Mat *img2, RobustMatcher rmatcher);
    void doReconstructionSweep (Reconstructor *rec, cv::Mat f, int image_rows, int image_cols, vector<cv::Point2f> *points1, vector<cv::Point2f> *points2);
    void adjustMatrixToLatestOrigin (cv::Mat *P, cv::Mat *B);

private slots:
    void on_pushButton_clicked  ();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    GLFrame *_GLFrame;

    cv::Mat _image;
    vector<cv::Mat> _vec_source_images;

    QSound *_ping;

    clock_t _start;
    clock_t _stop;

    MPointCloud *_pc;

    cv::Mat_<double> _O; // origin
    cv::Mat_<double> _B; // current base

protected:
    void closeEvent (QCloseEvent *evt);
};
#endif // MAINWINDOW_H
