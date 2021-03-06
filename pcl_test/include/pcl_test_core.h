#pragma once
#include<iostream>
#include <ros/ros.h>
#include <sstream>
//将传感器数据转换文pcl数据头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>//vgf头文件


#define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18//用于分割segments
#define SENSOR_HEIGHT 2.00//传感器高度

#define concentric_divider_distance_ 0.01 //0.1 meters default，用于分割bins
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //最大坡度：同一条射线上相邻两点的坡度阈值，max slope of the ground between points, degree
#define general_max_slope_ 5 //整个地面的坡度阈值max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2 //判断是否为地面点的阈值，即：同一条线的相邻两个点之间的距离小于0.2才进行地面非地面的判断

class PclTestCore
{

private:
  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_ground_, pub_no_ground_;
  //添加栅格后的点云输出
  ros::Publisher pub_filtered_points_;

  struct PointXYZIRTColor
  {
   
    pcl::PointXYZI point; //实例化XYZI点云
    
    float radius; //cylindric coords on XY Plane
    float theta;  //angle deg on XY plane

    size_t radial_div;     //index of the radial divsion to which this point belongs to
    size_t concentric_div; //index of the concentric division to which this points belongs to

    size_t original_index; //index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

  size_t radial_dividers_num_;//2250
  size_t concentric_dividers_num_;

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                        PointCloudXYZIRTColor &out_organized_points,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

  void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

public:
  PclTestCore(ros::NodeHandle &nh);
  ~PclTestCore();
  void Spin();
};