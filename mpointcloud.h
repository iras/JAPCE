#ifndef MPOINTCLOUD_H
#define MPOINTCLOUD_H

#include <vector>
#include <iostream>
using namespace std;

class MPointCloud
{

public:
    MPointCloud();
    ~MPointCloud();

    void addSegment (vector<vector<float> > *pc_segment);
    vector<vector<float> > *getSegment (unsigned int i);
    vector<vector<vector<float> > > *getPC (void);
    unsigned int size (void);

private:
    vector<vector<vector<float> > > _point_cloud;

};

#endif // MPOINTCLOUD_H
