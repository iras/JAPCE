#ifndef CAMERASHOT_H
#define CAMERASHOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
using namespace std;


class CameraShot
{

public:

    CameraShot (cv::Mat img);

    cv::Mat *getImage (void);

    cv::Mat_<double> *getP (void);

    vector<cv::KeyPoint> *getKeyPoints (void);

    void setKeyPoints (vector<cv::KeyPoint> kpts);

private:

    cv::Mat _image;

    vector<cv::KeyPoint> _keypoints;

    cv::Mat_<double> _P;

};

#endif // CAMERASHOT_H
