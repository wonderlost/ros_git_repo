#include "pcl_test/pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PclTestCore::point_cb, this);
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    ros::spin();
}



void PclTestCore::Spin(){

}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    //pcl::fromPCLPointCloud2(*in_cloud_ptr, *current_pc_ptr);

    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2, 0.2, 0.2);
    vg.filter(*filtered_pc_ptr);

    //=======剪裁点云-高度
    pcl::PointCloud<pcl::PointXYZI>::Ptr clip_above_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    clip_above(1.28, filtered_pc_ptr, clip_above_ptr);
    //=======剪裁近距离点
    pcl::PointCloud<pcl::PointXYZI>::Ptr removed_closept_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    remove_close_pt(1.5, clip_above_ptr, removed_closept_ptr);

    sensor_msgs::PointCloud2 pub_pc;

    //pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
    pcl::toROSMsg(*removed_closept_ptr, pub_pc);
    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}

void PclTestCore::clip_above(double clip_height, 
                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);

    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
            indices.indices.push_back(i);
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
    //ROS_INFO("clip the aboved points");
}

void PclTestCore::remove_close_pt(double min_distance, 
                                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true);
        cliper.filter(*out);

}