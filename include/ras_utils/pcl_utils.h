#ifndef PCL_UTILS_H
#define PCL_UTILS_H

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// =============================================================================
// ** Camera intrinsics (from /camera/depth_registered/camera_info topic)
#define FX              574.0527954101562
#define FY              574.0527954101562
#define FX_INV          1.0/FX
#define FY_INV          1.0/FY
#define CX              319.5
#define CY              239.5
#define IMG_ROWS        480
#define IMG_COLS        640
// =============================================================================

namespace PCL_Utils
{
void buildPointCloud(const cv::Mat &rgb_img, const cv::Mat &depth_img,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double scale_factor=1.0);
void transform2Dto3D(const cv::Point &p_in, const double &depth, pcl::PointXYZ &p_out);
void transformPoint(const pcl::PointXYZ &p_in, const Eigen::Matrix4f &transform, pcl::PointXYZ &p_out);
void transformPoint(const pcl::PointXY &p_in,  const Eigen::Matrix3f &transform, pcl::PointXY &p_out);

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
double euclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);
double euclideanDistance(const pcl::PointXY &p1, const pcl::PointXY &p2);
}

#endif // PCL_UTILS_H
