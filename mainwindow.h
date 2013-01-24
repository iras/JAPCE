#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <vector>
#include <QtGui/QMainWindow>
#include <QtGui/QFileDialog>
#include <QSound>

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
    
private slots:
    void on_pushButton_clicked ();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    GLFrame *GLFrame_1;

    cv::Mat image1;
    cv::Mat image2;

    QSound *_ping;

protected:
    void closeEvent (QCloseEvent *evt);
};
#endif // MAINWINDOW_H
