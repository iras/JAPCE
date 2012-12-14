#include "reconstructor.h"
using namespace std;


// Reconstructor's constructor
Reconstructor::Reconstructor() {}


cv::Mat_<double> *Reconstructor::getPCandidatesfromFundamentalMtx (cv::Mat_<double> Fund)
{
    cout << "- get P candidates from Fundamental Mtx" << endl;

    cv::Mat_<double> W = (cv::Mat_<double>(3,3) <<  0,-1,0,  1,0,0,  0,0,1); // orthogonal
    cv::Mat_<double> Z = (cv::Mat_<double>(3,3) <<  0,1,0,  -1,0,0,  0,0,0); // skew-symmetric

    // double m[3][3] = {{3117.75, 0, 1629.3}, {0, 3117.74, 1218.01}, {0, 0, 1}};
    // cv::Mat K   = cv::Mat (3, 3, CV_64F, m);
    // cv::Matx33d m (3117.75, 0, 1629.3,   0, 3117.74, 1218.01,  0,0,1);
    // cv::Mat_<double> K = (cv::Mat_<double>)m;
    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,  0,0,1);
    cv::Mat_<double> Kt  = K.t();
    //cv::Mat_<double> Ki  = K.inv();
    //cv::Mat_<double> Kti = Ki.t());

    cv::Mat_<double> Ess = Kt * (cv::Mat_<double>)Fund * K;

    cv::SVD svd (Ess);

    cv::Mat_<double> R1 = svd.u * W * svd.vt;
    cv::Mat_<double> R2 = svd.u * W.t() * svd.vt;

    cout << "- get P candidates from Fundamental Mtx - ROTATIONS" << endl;

    if (fabsf(cv::determinant(R1))-1.0 > 1e-07) {cout << "det(R1) != +-1.0, this is not a rotation matrix" << endl;}
    if (fabsf(cv::determinant(R2))-1.0 > 1e-07) {cout << "det(R2) != +-1.0, this is not a rotation matrix" << endl;}

    cv::Mat_<double> S  = svd.u * Z * svd.u.t();
    cv::Vec3d t = cv::Vec3d (S(0,2), S(1,2), S(2,2)); // this vector represents the rightmost column of the matrix S. Or, in other words, S*(u3)t.

    cout << "- get P candidates from Fundamental Mtx - TRANSLATIONS" << endl;

    static cv::Mat_<double> four_solutions_of_P[4];
    four_solutions_of_P[0] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2), t(0),  R1(1,0),R1(1,1),R1(1,2), t(1),  R1(2,0),R1(2,1),R1(2,2), t(2));
    four_solutions_of_P[1] = (cv::Mat_<double>(3,4)  <<  R1(0,0),R1(0,1),R1(0,2),-t(0),  R1(1,0),R1(1,1),R1(1,2),-t(1),  R1(2,0),R1(2,1),R1(2,2),-t(2));
    four_solutions_of_P[2] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2), t(0),  R2(1,0),R2(1,1),R2(1,2), t(1),  R2(2,0),R2(2,1),R2(2,2), t(2));
    four_solutions_of_P[3] = (cv::Mat_<double>(3,4)  <<  R2(0,0),R2(0,1),R2(0,2),-t(0),  R2(1,0),R2(1,1),R2(1,2),-t(1),  R2(2,0),R2(2,1),R2(2,2),-t(2));

    return four_solutions_of_P;
}



cv::Mat_<double> Reconstructor::pickTheRightP (cv::Mat_<double> P1, cv::Mat_<double> *list_possible_P2s, cv::Point2f point1, cv::Point2f point2)
{
    cout << "- pick the right P" << endl;

    int ind = 0;

    cv::Mat_<double> K = (cv::Mat_<double>(3,3) << 3117.75, 0, 1629.3,   0, 3117.74, 1218.01,  0,0,1);

    for (int i=0; i<4; i++)
    {
        // triangulate inliers and compute depth for each camera
        cout << *(list_possible_P2s+i) << endl;

        cv::Mat_<double> x1 = (cv::Mat_<double>(3,1) <<  point1.x, point1.y, 1.0);
        cv::Mat_<double> x2 = (cv::Mat_<double>(3,1) <<  point2.x, point2.y, 1.0);

        cv::Mat_<double> X = this->triangulate (x1, x2, P1, *(list_possible_P2s+i));

        cout << "- pick the right P : post triangulation" << endl;

        cv::Mat_<double> rx1 = K * P1 * X;

        cout << "- pick the right P : post rx1" << endl;

        cv::Mat_<double> rx2 = K * *(list_possible_P2s+i) * X;

        cout << "- pick the right P : post reprojection" << endl;

        if (rx1.data[2]>0 && rx2.data[2]>0)
        {
            ind = i;
            break;
        }
    }

    cout << "- pick the right P : end" << endl;

    return *(list_possible_P2s+ind);
}



// Point pair triangulation from least squares solution.
cv::Mat_<double> Reconstructor::triangulate (cv::Mat_<double> x1, cv::Mat_<double> x2, cv::Mat_<double> P1, cv::Mat_<double> P2)
{
    cout << "- triangulate" << endl;

    // negate projections, remember that the projections are 2d coords with a 1 as third value.
    double* x1p=x1.ptr<double>(0);   x1p[0]=-x1p[0]; x1p[1]=-x1p[1]; x1p[2]=-x1p[2];
    double* x2p=x2.ptr<double>(0);   x2p[0]=-x2p[0]; x2p[1]=-x2p[1]; x2p[2]=-x2p[2];
    //double* x1p=(double*)x1.data;   *x1p=-(*x1p); x1p++; *x1p=-(*x1p); x1p++; *x1p=-(*x1p);
    //double* x2p=(double*)x2.data;   *x2p=-(*x2p); x2p++; *x2p=-(*x2p); x2p++; *x2p=-(*x2p);

    // create 6x6 matrix which will then contain
    //      P1 -x1   0
    //      P2   0 -x2
    //
    cv::Mat_<double> M = cv::Mat_<double>::zeros (6,6);
    cv::Mat_<double> aux1 = M.colRange(0,4).rowRange(0,3);  P1.copyTo (aux1);
    cv::Mat_<double> aux2 = M.colRange(0,4).rowRange(3,6);  P2.copyTo (aux2);
    cv::Mat_<double> aux3 = M.colRange(4,5).rowRange(0,3);  x1.copyTo (aux3);
    cv::Mat_<double> aux4 = M.colRange(5,6).rowRange(3,6);  x2.copyTo (aux4);
    cout << "- triangulate : M 6x6 : " << M << endl;

    // solve the system
    //      P1 -x1   0      X
    //      P2   0 -x2      lambda1     =     0
    //                      lambda2
    // where achieving X is the only goal, not lambda1 neither lambda2.
    cv::SVD svd (M);
    cv::Mat_<double> vt = svd.vt;

    cv::Mat_<double> X = (cv::Mat_<double>(4,1) <<  vt.data[0], vt.data[1], vt.data[2], vt.data[3]);

    cout << "- triangulate : X " << X << endl;
    return X;
}
