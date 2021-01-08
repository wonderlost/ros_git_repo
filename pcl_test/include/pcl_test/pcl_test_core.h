#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <sensor_msgs/PointCloud2.h>

class PclTestCore
{
private:
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_filtered_points_;
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
    void remove_close_pt(double min_distance, 
                                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore() {}
    void Spin();
};