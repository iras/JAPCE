#include "camerashot.h"


CameraShot::CameraShot (cv::Mat img)
{
    _image = img;
}

cv::Mat *CameraShot::getImage (void)
{
    return &_image;
}

vector<cv::KeyPoint> *CameraShot::getKeyPoints (void)
{
    return &_keypoints;
}

void CameraShot::setKeyPoints (vector<cv::KeyPoint> kpts)
{
    _keypoints = kpts;
}
