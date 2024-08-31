/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-08-30 15:54:00
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-08-31 13:43:47
 * @FilePath: /undefined/home/cyun/learn_ws/lidar_fusion/src/lidar_fusion_param/include/lidar_fusion_param/lidar_fusion.h
 * @Description: lidar fusion use param
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <iostream>

class LidarFusion
{
public:
    LidarFusion()
    {
        int lidar_num;
        nh_.getParam("lidar_num", lidar_num);

        // 遍历每一个雷达进行转换
        for (int i = 1; i <= lidar_num; ++i)
        {
            std::string lidar_topic;
            nh_.getParam("lidar_topic" + std::to_string(i), lidar_topic);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/x", transform_params_[i - 1][0]);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/y", transform_params_[i - 1][1]);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/z", transform_params_[i - 1][2]);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/roll", transform_params_[i - 1][3]);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/pitch", transform_params_[i - 1][4]);
            nh_.getParam("lidar" + std::to_string(i) + "_to_lidar_link/yaw", transform_params_[i - 1][5]);

            Eigen::Affine3d transform = Eigen::Affine3d::Identity();
            transform.translation() << transform_params_[i - 1][0], transform_params_[i - 1][1], transform_params_[i - 1][2];
            // 旋转
            transform.rotate(Eigen::AngleAxisd(transform_params_[i - 1][3], Eigen::Vector3d::UnitX()));
            transform.rotate(Eigen::AngleAxisd(transform_params_[i - 1][4], Eigen::Vector3d::UnitY()));
            transform.rotate(Eigen::AngleAxisd(transform_params_[i - 1][5], Eigen::Vector3d::UnitZ()));

            // 传入
            transforms_.push_back(transform);

            // 订阅雷达
            subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, boost::bind(&LidarFusion::lidarCallback, this, _1, i - 1)));
        }

        // 融合后的话题和frame:
        nh_.getParam("fusion_topic", fusion_topic_);
        nh_.getParam("fusion_frame", fusion_frame_);

        // 初始化pub
        fusion_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(fusion_topic_, 10);
    }

private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subs_;
    ros::Publisher fusion_pub_;
    std::vector<Eigen::Affine3d> transforms_;
    std::vector<sensor_msgs::PointCloud2> clouds_;
    double transform_params_[2][6];
    std::string fusion_topic_, fusion_frame_;

    // 处理每一个雷达的数据
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int lidar_index)
    {
        // 使用 Eigen 库创建转换矩阵
        Eigen::Affine3f transform = transforms_[lidar_index].cast<float>();

        // 打印转换矩阵以确认
        std::cout << "Transformation Matrix for Lidar " << lidar_index + 1 << ":" << std::endl;
        std::cout << transform.matrix() << std::endl;

        // 将点云转换到目标坐标系
        sensor_msgs::PointCloud2 transformed_cloud_msg;
        pcl_ros::transformPointCloud(transform.matrix(), *msg, transformed_cloud_msg);

        // 将转换后的点云存储在云列表中
        if (clouds_.size() <= lidar_index)
        {
            clouds_.resize(lidar_index + 1);
        }
        clouds_[lidar_index] = transformed_cloud_msg;

        // 检查是否所有雷达数据都已经到达
        if (std::all_of(clouds_.begin(), clouds_.end(), [](const sensor_msgs::PointCloud2& c) { return c.data.size() > 0; }))
        {
            std::cout << "Publishing fused point cloud" << std::endl;

            // 合并所有点云
            sensor_msgs::PointCloud2 fused_cloud = clouds_[0];
            for (size_t i = 1; i < clouds_.size(); ++i)
            {
                pcl::concatenatePointCloud(fused_cloud, clouds_[i], fused_cloud);
            }

            // 将合并的点云发布
            fused_cloud.header.frame_id = fusion_frame_;
            fusion_pub_.publish(fused_cloud);

            // 清除点云
            clouds_.clear();
        }
    }
};