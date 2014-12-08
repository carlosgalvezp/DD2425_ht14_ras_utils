#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <ras_utils/ras_names.h>

// ROS
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Eigen
#include <Eigen/Core>

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
void buildPointCloud(const cv::Mat &depth_img, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, double scale_factor=1.0);

void transform2Dto3D(const cv::Point &p_in, const double &depth, pcl::PointXYZ &p_out);
void transformPoint(const pcl::PointXYZ &p_in, const Eigen::Matrix4f &transform, pcl::PointXYZ &p_out);
void transformPoint(const pcl::PointXY &p_in,  const Eigen::Matrix3f &transform, pcl::PointXY &p_out);

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
double euclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);
double euclideanDistance(const pcl::PointXY &p1, const pcl::PointXY &p2);

bool readTransform(const std::string &frame_from,
                   const std::string &frame_to,
                   const tf::TransformListener &tf_listener,
                   tf::Transform &tf);

void convertEigen4x4ToTransform(const Eigen::Matrix4f &tf_eigen, tf::Transform &tf_transform);
void convertTransformToEigen4x4(const tf::Transform &tf_transform, Eigen::Matrix4f &tf_eigen);

}

#endif // PCL_UTILS_H
