#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <vector>
#include <iostream>
#include <pcsegment.h>
#include <camerashot.h>
#include <opencv2/core/core.hpp>

using namespace std;


class PointCloud
{

public:

    PointCloud();

    unsigned int size (void);
    void addPCSegment (CameraShot *cs1, CameraShot *cs2);
    PCSegment *getPCSegment (unsigned int i);

private:

    vector<PCSegment> _point_cloud;

};

#endif // POINTCLOUD_H
