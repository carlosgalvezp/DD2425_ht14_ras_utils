#include <ras_utils/pcl_utils.h>

namespace PCL_Utils
{

void buildPointCloud(const cv::Mat &rgb_img, const cv::Mat &depth_img,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double scale_factor)
{
    if(cloud_out != 0)
    {
        int height = rgb_img.rows * scale_factor;
        int width  = rgb_img.cols * scale_factor;
        int step = 1.0 / scale_factor;

        // ** Construct point cloud
        cloud_out->resize(height*width);

        std::size_t actual_size = 0;
        for (unsigned int v = 0; v < rgb_img.rows; v+=step)
        {
            for (unsigned int u = 0; u < rgb_img.cols; u+=step)
            {
                float z = depth_img.at<float>(v, u);
                if(z != 0 && !std::isnan(z))
                {
                    const cv::Vec3b& c = rgb_img.at<cv::Vec3b>(v, u);
                    pcl::PointXYZRGB& pt = cloud_out->points[actual_size++];

                    pt.x = z * ((u - CX) * FX_INV);
                    pt.y = z * ((v - CY) * FY_INV);
                    pt.z = z;
                    pt.r = c[0];
                    pt.g = c[1];
                    pt.b = c[2];
                }
            }
        }
        cloud_out->resize(actual_size);
        cloud_out->width = actual_size;
        cloud_out->height = 1;
        cloud_out->is_dense = true;
        cloud_out->header.frame_id = COORD_FRAME_CAMERA_LINK;
    }
    else
    {
        std::cout << "[Build Point Cloud] The input Ptr is 0"<<std::endl;
    }
}

void buildPointCloud(const cv::Mat &depth_img,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, double scale_factor)
{
    if(cloud_out != 0)
    {
        int height = depth_img.rows * scale_factor;
        int width  = depth_img.cols * scale_factor;
        int step = 1.0 / scale_factor;

        // ** Construct point cloud
        cloud_out->resize(height*width);

        std::size_t actual_size = 0;
        for (unsigned int v = 0; v < depth_img.rows; v+=step)
        {
            for (unsigned int u = 0; u < depth_img.cols; u+=step)
            {
                float z = depth_img.at<float>(v, u);
                if(z != 0 && !std::isnan(z))
                {
                    pcl::PointXYZ& pt = cloud_out->points[actual_size++];

                    pt.x = z * ((u - CX) * FX_INV);
                    pt.y = z * ((v - CY) * FY_INV);
                    pt.z = z;
                }
            }
        }
        cloud_out->resize(actual_size);
        cloud_out->width = actual_size;
        cloud_out->height = 1;
        cloud_out->is_dense = true;
        cloud_out->header.frame_id = COORD_FRAME_CAMERA_LINK;
    }
    else
    {
        std::cout << "[Build Point Cloud] The input Ptr is 0"<<std::endl;
    }
}

void transform2Dto3D(const cv::Point &p_in, const double &depth, pcl::PointXYZ &p_out)
{
    p_out.x = depth * ((p_in.x - CX) * FX_INV);
    p_out.y = depth * ((p_in.y - CY) * FY_INV);
    p_out.z = depth;
}

void transformPoint(const pcl::PointXYZ &p_in, const Eigen::Matrix4f &transform, pcl::PointXYZ &p_out)
{
    Eigen::Vector4f p_in_norm, p_out_norm;
    p_in_norm << p_in.x, p_in.y, p_in.z, 1.0;

    p_out_norm = transform * p_in_norm;

    p_out.x = p_out_norm(0,0);
    p_out.y = p_out_norm(1,0);
    p_out.z = p_out_norm(2,0);
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->spin();

}
void transformPoint(const pcl::PointXY &p_in, const Eigen::Matrix3f &transform, pcl::PointXY &p_out)
{
    Eigen::Vector3f p_in_norm, p_out_norm;
    p_in_norm << p_in.x, p_in.y, 1.0;

    p_out_norm = transform * p_in_norm;

    p_out.x = p_out_norm(0,0);
    p_out.y = p_out_norm(1,0);
}

double euclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

double euclideanDistance(const pcl::PointXY &p1, const pcl::PointXY &p2)
{
    return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}


bool readTransform(const std::string &frame_from,
                   const std::string &frame_to,
                   const tf::TransformListener &tf_listener,
                   tf::Transform &tf)
{
    tf::StampedTransform transform;
    try
    {
        tf_listener.lookupTransform(frame_from, frame_to,ros::Time(0), transform);
        tf.setRotation(transform.getRotation());
        tf.setOrigin(transform.getOrigin());
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("FAILED TO READ TRANSFORM BETWEEN %s and %s", frame_from.c_str(), frame_to.c_str());
        return false;
    }
}

void convertEigen4x4ToTransform(const Eigen::Matrix4f &tf_eigen, tf::Transform &tf_transform)
{
    tf::Vector3 origin;
    tf::Matrix3x3 tf3d;
    origin.setValue(tf_eigen(0,3), tf_eigen(1,3), tf_eigen(2,3));

    tf3d.setValue(tf_eigen(0,0),  tf_eigen(0,1),  tf_eigen(0,2),
                  tf_eigen(1,0),  tf_eigen(1,1),  tf_eigen(1,2),
                  tf_eigen(2,0),  tf_eigen(2,1),  tf_eigen(2,2));

    tf::Quaternion q;
    tf3d.getRotation(q);

    tf_transform.setOrigin(origin);
    tf_transform.setRotation(q);
}

void convertTransformToEigen4x4(const tf::Transform &tf_transform, Eigen::Matrix4f &tf_eigen)
{
    tf::Vector3 origin = tf_transform.getOrigin();
    tf::Matrix3x3 rotation(tf_transform.getRotation());

    tf_eigen << rotation[0][0] , rotation[0][1] , rotation[0][2] , origin[0],
                rotation[1][0] , rotation[1][1] , rotation[1][2] , origin[1],
                rotation[2][0] , rotation[2][1] , rotation[2][2] , origin[2],
                0              , 0              , 0              , 1;
}

}
