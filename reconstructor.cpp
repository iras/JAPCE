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
    cout << "- get P candidates from Fundamental Mtx" << endl;

    cv::Mat_<double> W = (cv::Mat_<double>(3,3) <<  0,-1,0,  1,0,0,  0,0,1); // orthogonal
    cv::Mat_<double> Z = (cv::Mat_<double>(3,3) <<  0,1,0,  -1,0,0,  0,0,0); // skew-symmetric

    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,   0, 0, 1);
    cv::Mat_<double> Kt  = K.t();
    //cv::Mat_<double> Ki  = K.inv();
    //cv::Mat_<double> Kti = Ki.t());

    cv::Mat_<double> Ess = Kt * Fund * K;
    cv::SVD svd (Ess);

    cv::Mat_<double> U  = svd.u;

    cv::Mat_<double> R1 = U * W * svd.vt;
    cv::Mat_<double> R2 = U * W.t() * svd.vt;

    if (fabsf(cv::determinant (R1))-1.0 > 1e-07) {cout << "det(R1) != +-1.0, this is not a rotation matrix" << endl;}
    if (fabsf(cv::determinant (R2))-1.0 > 1e-07) {cout << "det(R2) != +-1.0, this is not a rotation matrix" << endl;}

    cv::Vec3d u3 = cv::Vec3d (U(0,2), U(1,2), U(2,2)); // this vector represents the rightmost column of the matrix U. Or, in other words (u3)^t.

    static cv::Mat_<double> four_solutions_of_P[4];
    four_solutions_of_P[0] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2), u3(0),  R1(1,0),R1(1,1),R1(1,2), u3(1),  R1(2,0),R1(2,1),R1(2,2), u3(2));
    four_solutions_of_P[1] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2),-u3(0),  R1(1,0),R1(1,1),R1(1,2),-u3(1),  R1(2,0),R1(2,1),R1(2,2),-u3(2));
    four_solutions_of_P[2] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2), u3(0),  R2(1,0),R2(1,1),R2(1,2), u3(1),  R2(2,0),R2(2,1),R2(2,2), u3(2));
    four_solutions_of_P[3] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2),-u3(0),  R2(1,0),R2(1,1),R2(1,2),-u3(1),  R2(2,0),R2(2,1),R2(2,2),-u3(2));

    return four_solutions_of_P;
}



cv::Mat_<double> Reconstructor::pickTheRightP (cv::Mat_<double> P1, cv::Mat_<double> *list_possible_P2s, vector<cv::Point2f> points1, vector<cv::Point2f> points2)
{
    cout << "- pick the right P" << endl;

    vector <double> rx1_list;
    vector <double> rx2_list;
    int index = 0;
    float max = 0;
    cv::Mat_<double> X;
    cv::Mat_<double> x1;
    cv::Mat_<double> x2;
    cv::Mat_<double> rx1;
    cv::Mat_<double> rx2;
    double sum_rx1;
    double sum_rx2;
    unsigned int sum_rx1_greater_than_0;
    unsigned int sum_rx2_greater_than_0;
    double* tmpp;

    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,   0, 0, 1);

    for (unsigned int i=0; i<4; i++)
    {
        // triangulate inliers and compute depth for each camera
        cout << *(list_possible_P2s+i) << endl;

        rx1_list.clear();
        rx2_list.clear();

        for (unsigned int j=0; j<points1.size(); j++)
        {
            x1 = (cv::Mat_<double>(3,1) <<  points1[j].x, points1[j].y, 1.0);
            x2 = (cv::Mat_<double>(3,1) <<  points2[j].x, points2[j].y, 1.0);

            X = this->triangulate (x1, x2, P1, *(list_possible_P2s+i));

            rx1 = K * P1 * X;
            tmpp = rx1.ptr<double>(0);
            rx1_list.push_back (tmpp[2]);


            rx2 = K * *(list_possible_P2s+i) * X;
            tmpp = rx1.ptr<double>(0);
            rx2_list.push_back (tmpp[2]);
        }

        // sums
        sum_rx1=0;  for (vector<double>::iterator k = rx1_list.begin(); k!=rx1_list.end(); ++k)  sum_rx1 += *k;
        sum_rx2=0;  for (vector<double>::iterator k = rx2_list.begin(); k!=rx2_list.end(); ++k)  sum_rx2 += *k;

        cout << i << " --- sum_rx1 : " << sum_rx1 << " ---sum_rx2 : " << sum_rx2 << endl;
        cout << i << " ---    size : " << rx1_list.size() << " ---    size : " << rx2_list.size() << endl;

        sum_rx1_greater_than_0 = 0;  if (sum_rx1>0) sum_rx1_greater_than_0 = 1;
        sum_rx2_greater_than_0 = 0;  if (sum_rx2>0) sum_rx2_greater_than_0 = 1;

        if (sum_rx1_greater_than_0 + sum_rx2_greater_than_0 > max)
        {
            index = i;
            max = sum_rx1_greater_than_0 + sum_rx2_greater_than_0;
        }
    }

    _index = index;

    return *(list_possible_P2s+index);
}



// Point pair triangulation from least squares solution.
cv::Mat_<double> Reconstructor::triangulate (cv::Mat_<double> x1, cv::Mat_<double> x2, cv::Mat_<double> P1, cv::Mat_<double> P2)
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
    cv::Mat_<double> aux1 = M.colRange(0,4).rowRange(0,3);  P1.copyTo (aux1);
    cv::Mat_<double> aux2 = M.colRange(0,4).rowRange(3,6);  P2.copyTo (aux2);
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



// getters/setters

int Reconstructor::getIndex (void) {return _index;}
