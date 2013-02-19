#include "pointcloud.h"


PointCloud::PointCloud()
{
    _point_cloud.clear();
}


unsigned int PointCloud::size (void)
{
    return _point_cloud.size ();
}


void PointCloud::addPCSegment (CameraShot *cs1, CameraShot *cs2)
{
    PCSegment pc_segment = PCSegment (cs1, cs2);

    _point_cloud.push_back (pc_segment);
}


PCSegment *PointCloud::getPCSegment (unsigned int i)
{
    return &(_point_cloud[i]);
}

