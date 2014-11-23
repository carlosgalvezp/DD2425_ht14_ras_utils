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

        int i = 0;
        for (unsigned int v = 0; v < rgb_img.rows; v+=step)
        {
            for (unsigned int u = 0; u < rgb_img.cols; u+=step)
            {
                float z_m = depth_img.at<float>(v, u);
                const cv::Vec3b& c = rgb_img.at<cv::Vec3b>(v, u);
                pcl::PointXYZRGB& pt = cloud_out->points[i++];

                pt.x = z_m * ((u - CX) * FX_INV);
                pt.y = z_m * ((v - CY) * FY_INV);
                pt.z = z_m;
                pt.r = c[0];
                pt.g = c[1];
                pt.b = c[2];
            }
        }
        cloud_out->width = width;
        cloud_out->height = height;
        cloud_out->is_dense = true;
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

}
