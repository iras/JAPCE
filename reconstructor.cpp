#include "reconstructor.h"
using namespace std;
#include <numeric>


// Reconstructor's constructor
Reconstructor::Reconstructor()
{
    _index = -1;
}


cv::Mat_<double> *Reconstructor::getPCandidatesfromFundamentalMtx (cv::Mat_<double> Fund)
{
    cv::Mat_<double> W = (cv::Mat_<double>(3,3) <<  0,-1,0,  1,0,0,  0,0,1); // orthogonal
    cv::Mat_<double> Z = (cv::Mat_<double>(3,3) <<  0,1,0,  -1,0,0,  0,0,0); // skew-symmetric

    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,   0, 0, 1);
    cv::Mat_<double> Kt  = K.t();

    cv::Mat_<double> Ess = Kt * Fund * K;
    cv::SVD svd (Ess);

    cv::Mat_<double> U  = svd.u;
    cv::Mat_<double> Vt = svd.vt;

    if (cv::determinant (U * Vt) < 0)
    {
        cout << "INVERSION" << endl;
        Vt = -Vt;
    }

    cv::Mat_<double> R1 = U * W * Vt;
    cv::Mat_<double> R2 = U * W.t() * Vt;

    cv::Vec3d u3 = cv::Vec3d (U(0,2), U(1,2), U(2,2)); // this vector represents the rightmost column of the matrix U. Or, in other words (u3)^t.

    static cv::Mat_<double> four_solutions_of_P[4];
    four_solutions_of_P[0] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2), u3(0),  R1(1,0),R1(1,1),R1(1,2), u3(1),  R1(2,0),R1(2,1),R1(2,2), u3(2));
    four_solutions_of_P[1] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2),-u3(0),  R1(1,0),R1(1,1),R1(1,2),-u3(1),  R1(2,0),R1(2,1),R1(2,2),-u3(2));
    four_solutions_of_P[2] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2), u3(0),  R2(1,0),R2(1,1),R2(1,2), u3(1),  R2(2,0),R2(2,1),R2(2,2), u3(2));
    four_solutions_of_P[3] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2),-u3(0),  R2(1,0),R2(1,1),R2(1,2),-u3(1),  R2(2,0),R2(2,1),R2(2,2),-u3(2));

    return four_solutions_of_P;
}



cv::Mat_<double> Reconstructor::calculateP2 (cv::Mat_<double> P1,
                                             cv::Mat_<double> *list_possible_P2s,
                                             vector<cv::Point2f> points1,
                                             vector<cv::Point2f> points2)
{
    cv::Mat_<double> X, x1, x2, reproject_x1, reproject_x2;
    double sum_rx1, sum_rx2;
    double* tmpp;

    _index = 0;
    double max = 0;
    unsigned int v;
    for (unsigned int i=0; i<4; i++)
    {
        // triangulate inliers + outliers and compute depth for each camera
        //cout << *(list_possible_P2s + i) << endl;

        sum_rx1=0;
        sum_rx2=0;

        for (unsigned int j=0; j<points1.size(); j++)
        {
            x1 = (cv::Mat_<double>(3,1) <<  points1[j].x, points1[j].y, 1.0);
            x2 = (cv::Mat_<double>(3,1) <<  points2[j].x, points2[j].y, 1.0);

            X = this->triangulate (x1, x2, P1, *(list_possible_P2s + i));

            reproject_x1 = P1 * X;
            tmpp = reproject_x1.ptr<double>(0);
            v = tmpp[2]>0 ? 1 : 0;
            sum_rx1 += v;  // increment sum_rx1 of 1 if tmpp[2] is positive.

            reproject_x2 = *(list_possible_P2s + i) * X;
            tmpp = reproject_x2.ptr<double>(0);
            v = tmpp[2]>0 ? 1 : 0;
            sum_rx2 += v;  // increment sum_rx2 of 1 if tmpp[2] is positive.
        }

        cout << i << " • sum_rx1=" << sum_rx1 << " sum_rx2=" << sum_rx2 << " Total=" << sum_rx1+sum_rx2 << endl;

        if (sum_rx1 + sum_rx2 > max)
        {
            _index = i;
            max = sum_rx1 + sum_rx2;
        }
    }

    return *(list_possible_P2s + _index);
}



// Point pair triangulation from least squares solution.
cv::Mat_<double> Reconstructor::triangulate (cv::Mat_<double> x1,
                                             cv::Mat_<double> x2,
                                             cv::Mat_<double> P1,
                                             cv::Mat_<double> P2)
{
    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,   0, 0, 1);
    cv::Mat_<double> Kinv = K.inv();

    cv::Mat_<double> x1c = Kinv * x1.clone();
    cv::Mat_<double> x2c = Kinv * x2.clone();

    // negate projections, remember that the projections are 2d coords with an additional value = 1 at the end.
    double* x1p = x1c.ptr<double>(0);   x1p[0]=-x1p[0]; x1p[1]=-x1p[1]; x1p[2]=-x1p[2];
    double* x2p = x2c.ptr<double>(0);   x2p[0]=-x2p[0]; x2p[1]=-x2p[1]; x2p[2]=-x2p[2];

    // create 6x6 matrix which will then contain
    //      P1 -x1   0
    //      P2   0 -x2
    //
    cv::Mat_<double> M = cv::Mat_<double>::zeros (6,6);
    cv::Mat_<double> aux1 = M.colRange(0,4).rowRange(0,3);  P1.copyTo  (aux1);
    cv::Mat_<double> aux2 = M.colRange(0,4).rowRange(3,6);  P2.copyTo  (aux2);
    cv::Mat_<double> aux3 = M.colRange(4,5).rowRange(0,3);  x1c.copyTo (aux3);
    cv::Mat_<double> aux4 = M.colRange(5,6).rowRange(3,6);  x2c.copyTo (aux4);
    //cout << "- triangulate : M 6x6 : " << M << endl;

    // solve the system
    //      P1 -x1   0      X
    //      P2   0 -x2      lambda1     =     0
    //                      lambda2
    // where X is the only data of interest.
    cv::SVD svd (M);
    cv::Mat_<double> vt = svd.vt;

    double rcpr = 1/vt(5,3);
    cv::Mat_<double> X = (cv::Mat_<double>(4,1)  <<  vt(5,0)*rcpr, vt(5,1)*rcpr, vt(5,2)*rcpr, vt(5,3)*rcpr);

    return X;
}



void Reconstructor::doTriangulationSweep (cv::Mat_<double> *P1,
                                          cv::Mat_<double> *P2,
                                          vector<cv::Point2f> *points1,
                                          vector<cv::Point2f> *points2,
                                          vector<cv::Point3d> *vector3d)
{
    cv::Mat_<double> x1, x2;
    cv::Mat_<double> X; // 3D coordinates for a reconstructed point.
    double* p;
    cv::Point3d point;

    for (unsigned int i=0; i < points1->size(); i++)
    {
        x1 = (cv::Mat_<double>(3,1) <<  (points1->at(i)).x, (points1->at(i)).y, 1.0);
        x2 = (cv::Mat_<double>(3,1) <<  (points2->at(i)).x, (points2->at(i)).y, 1.0);

        X = this->triangulate (x1, x2, *P1, *P2);

        p = X.ptr<double>(0);
        point = cv::Point3d (p[0], p[1], p[2]);
        vector3d->push_back (point); // push the vector point into the vector pc_segment.
    }
}



// getters/setters

int Reconstructor::getIndex (void) {return _index;}
