#include "pcsegment.h"


PCSegment::PCSegment (CameraShot *cs1, CameraShot *cs2)
{
    _cs1 = cs1;
    _cs2 = cs2;

    _vectX.clear ();
    _matches.clear ();

    _points1.clear ();
    _points2.clear ();
}

vector<cv::DMatch> *PCSegment::getMatches (void)
{
    return &_matches;
}

vector<cv::Point3d> *PCSegment::getVectorX (void)
{
    return &_vectX;
}

vector<cv::Point2f> *PCSegment::getPoints1 (void)
{
    return &_points1;
}

vector<cv::Point2f> *PCSegment::getPoints2 (void)
{
    return &_points2;
}

CameraShot *PCSegment::getCamShot1 (void)
{
    return _cs1;
}

CameraShot *PCSegment::getCamShot2 (void)
{
    return _cs2;
}

void PCSegment::squeezePointsVectorsOutOfMatches (void)
{
    float x1, y1, x2, y2;
    vector<cv::KeyPoint> *keypoints1 = _cs1->getKeyPoints ();
    vector<cv::KeyPoint> *keypoints2 = _cs2->getKeyPoints ();

    for (vector<cv::DMatch>::const_iterator it=(_matches).begin (); it!=(_matches).end (); ++it)
    {
        x1 = (*keypoints1) [it->queryIdx].pt.x;
        y1 = (*keypoints1) [it->queryIdx].pt.y;
        _points1.push_back (cv::Point2f (x1,y1));

        x2 = (*keypoints2) [it->trainIdx].pt.x;
        y2 = (*keypoints2) [it->trainIdx].pt.y;
        _points2.push_back (cv::Point2f (x2,y2));
    }
}
