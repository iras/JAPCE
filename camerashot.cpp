#include "camerashot.h"


CameraShot::CameraShot (cv::Mat img)
{
    _image = img;
    _P = (cv::Mat_<double>(3,4) <<  1,0,0,0,  0,1,0,0,  0,0,1,0); // init with 3x4 identity transform
}

cv::Mat *CameraShot::getImage (void)
{
    return &_image;
}

cv::Mat_<double> *CameraShot::getP (void)
{
    return &_P;
}

vector<cv::KeyPoint> *CameraShot::getKeyPoints (void)
{
    return &_keypoints;
}

void CameraShot::setKeyPoints (vector<cv::KeyPoint> kpts)
{
    _keypoints = kpts;
}
