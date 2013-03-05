#ifndef PCSEGMENT_H
#define PCSEGMENT_H

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <camerashot.h>

using namespace std;


// Point Cloud Segment class.
class PCSegment
{

public:

    PCSegment (CameraShot *cs1, CameraShot *cs2);

    vector<cv::DMatch>  *getMatches (void);
    vector<cv::Point3d> *getVectorX (void);

    vector<cv::Point2f> *getPoints1 (void);
    vector<cv::Point2f> *getPoints2 (void);

    CameraShot *getCamShot1 (void);
    CameraShot *getCamShot2 (void);

    void squeezePointsVectorsOutOfMatches (void);

private:

    CameraShot *_cs1;
    CameraShot *_cs2;

    vector<cv::Point3d> _vectX;
    vector<cv::DMatch>  _matches;

    vector<cv::Point2f> _points1;
    vector<cv::Point2f> _points2;

};

#endif // PCSEGMENT_H
