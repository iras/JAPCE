#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/photo/photo.hpp"
#include <opencv2/legacy/legacy.hpp>

using namespace std;



class Reconstructor
{
public:
    Reconstructor();

    cv::Mat_<double> *getPCandidatesfromFundamentalMtx (cv::Mat_<double> Fund);

    cv::Mat_<double> calculateP2 (cv::Mat_<double> P1,
                                  cv::Mat_<double> *list_possible_P2s,
                                  vector<cv::Point2f> points1,
                                  vector<cv::Point2f> points2);

    cv::Mat_<double> triangulate (cv::Mat_<double> x1,
                                  cv::Mat_<double> x2,
                                  cv::Mat_<double> P1,
                                  cv::Mat_<double> P2);

    void doTriangulationSweep    (cv::Mat_<double> *P1,
                                  cv::Mat_<double> *P2,
                                  vector<cv::Point2f> *points1,
                                  vector<cv::Point2f> *points2,
                                  vector<cv::Point3d> *pc_segment);

    int getIndex (void);

private:

    int _index;
};

#endif // RECONSTRUCTOR_H


