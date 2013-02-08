#include "mpointcloud.h"


MPointCloud::MPointCloud()
{
    _point_cloud.clear();
}

vector<vector<vector<float> > > *MPointCloud::getPC (void)
{
    return &_point_cloud;
}

unsigned int MPointCloud::size (void)
{
    return _point_cloud.size ();
}

void MPointCloud::addSegment (vector<vector<float> > *pc_segment)
{
    // copy point cloud segment
    vector<vector<float> > meta_temp;
    for (vector<vector <float> >::iterator i=pc_segment->begin(); i!=pc_segment->end(); ++i)  {meta_temp.push_back(*i);}

    // push the new segment in the vector containing the point cloud segments.
     _point_cloud.push_back (meta_temp);
}

vector<vector<float> > *MPointCloud::getSegment (unsigned int i)
{
    vector<vector<vector<float> > > * ptrvec;
    ptrvec = &_point_cloud;

    return &(ptrvec->at (i));
}
