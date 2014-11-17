#ifndef TYPES_H
#define TYPES_H

#include <pcl/features/fpfh.h>

namespace RAS_Utils{

    // 3D descriptor
    typedef pcl::FPFHSignature33                                               DescriptorType;
    typedef pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, DescriptorType> DescriptorExtractor;
}

#endif // TYPES_H
