#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>


#include <sensor_msgs/PointCloud2.h>


              double length_x_clipped = 0;
              double length_y_clipped = 0;
              double length_z_clipped = 0;

class PclTestCore
{   

  public:
  // 初始化节点
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_odom_boxMapping;
    ros::Subscriber sub_ladar_map_feats;

    ros::Publisher pub_filtered_points_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_raw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_box;




    // ros::Publisher pubFilteredPoints_short_mapping_;
    // 降采样

    // void point_clipping(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //     pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    //     pcl::VoxelGrid<pcl::PointXYZI> vg;

    //     vg.setInputCloud(current_pc_ptr);
    //     float data_x = 1.0f;
    //     vg.setLeafSize(data_x, data_x, data_x);
    //     vg.filter(*filtered_pc_ptr);

    //     sensor_msgs::PointCloud2 pub_pc;
    //     pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    //     pub_pc.header = in_cloud_ptr->header;

    //     pub_filtered_points_.publish(pub_pc);
    // }

      // 降采样+横截面截取
        void point_box_clipping(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
        current_pc_raw = current_pc_ptr;

        // pcl::VoxelGrid<pcl::PointXYZI> vg;
        // vg.setInputCloud(current_pc_ptr);
        // float data_x = 1.0f;
        // vg.setLeafSize(data_x, data_x, data_x);
        // vg.filter(*filtered_pc_ptr);

        typedef pcl::PointXYZI PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;

        PointT minValues, maxValues;
        pcl::getMinMax3D(*current_pc_ptr, minValues, maxValues);//得到输入点云x，y，z三个轴上的最小值最大值

        // double y_middle = (minValues.y + maxValues.y) * 0.5;
        // double x_middle = (minValues.y + maxValues.y) * 0.5;
        // double z_middle = (minValues.z + maxValues.z) * 0.5;

         pcl::PassThrough<PointT> pass;
          pass.setInputCloud(current_pc_ptr);
          pass.setFilterFieldName("z");
          // pass.setFilterLimits(z_middle - length_z_clipped * 0.9, z_middle + length_z_clipped * 0.1);
          pass.setFilterLimits(-length_z_clipped * 0.5,  length_z_clipped * 0.5);
      //    pass.setNegative(false);
          pass.filter(*current_pc_ptr);

          pass.setInputCloud(current_pc_ptr);
          pass.setFilterFieldName("y");
          // pass.setFilterLimits(y_middle - length_y_clipped * 0.5, y_middle + length_y_clipped * 0.5);
          pass.setFilterLimits( -length_y_clipped * 0.5,  length_y_clipped * 0.5);

      //    pass.setNegative(false);
          pass.filter(*current_pc_ptr);

          pass.setInputCloud(current_pc_ptr);
          pass.setFilterFieldName("x");
          pass.setFilterLimits(-length_x_clipped * 0.5,  length_x_clipped * 0.5);
          // pass.setFilterLimits(x_middle - length_x_clipped * 0.5, x_middle + length_x_clipped * 0.5);
      //    pass.setNegative(false);
          pass.filter(*filtered_pc_ptr);
          filtered_pc_box = filtered_pc_ptr;
        sensor_msgs::PointCloud2 pub_pc;
        pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
        pub_pc.header = in_cloud_ptr->header;
        pub_filtered_points_.publish(pub_pc);
    }
    void get_pose(){

    }

    


  public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
    void Spin(){
    }
};

 PclTestCore::PclTestCore(ros::NodeHandle &nh){
    // sub_point_cloud_ = nh.subscribe("/cloud_registered",100000, &PclTestCore::point_clipping, this);
    // sub_point_cloud_ = nh.subscribe("/cloud_registered",100000, &PclTestCore::point_box_clipping, this);
    // sub_point_cloud_ = nh.subscribe("/cloud_registered_body",100000, &PclTestCore::point_box_clipping, this);
    // sub_odom_boxMapping = nh.subscribe("/Odometry",100000, &PclTestCore::get_pose, this);
    sub_ladar_map_feats = nh.subscribe("/Laser_map",100000, &PclTestCore::point_box_clipping, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100000);
    ros::spin();
}


PclTestCore::~PclTestCore(){}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "middle_node");




    ros::NodeHandle nh;
    nh.param<double>("length_x_clipped",length_x_clipped,5.0);//平面点
    nh.param<double>("length_y_clipped",length_y_clipped,5.0);//平面点
    nh.param<double>("length_z_clipped",length_z_clipped,0.3);//平面点

    PclTestCore core(nh);
    return 0;
}