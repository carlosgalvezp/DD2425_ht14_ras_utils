#ifndef TYPES_H
#define TYPES_H

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
namespace RAS_Utils{

    // 3D descriptor
    typedef pcl::PFHSignature125                                               DescriptorType;
    typedef pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, DescriptorType> DescriptorExtractor;
}

#endif // TYPES_H
