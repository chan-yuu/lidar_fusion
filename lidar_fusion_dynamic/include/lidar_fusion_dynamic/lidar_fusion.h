/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-08-30 15:54:00
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-08-31 16:45:58
 * @FilePath: /undefined/home/cyun/learn_ws/lidar_fusion/src/lidar_fusion_dynamic/include/lidar_fusion_dynamic/lidar_fusion.h
 * @Description: lidar fusion use param
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <dynamic_reconfigure/server.h>
#include <lidar_fusion_dynamic/LidarFusionConfigConfig.h> // 动态配置头文件

class LidarFusion
{
public:
    LidarFusion()
    {
        // 获取参数服务器中的雷达数量
        if (!nh_.getParam("lidar_num", lidar_num_))
        {
            ROS_ERROR("Failed to get param 'lidar_num'. Please make sure it is set in the launch file or parameter server.");
            ros::shutdown();
            return;
        }

        // 初始化转换矩阵
        if (lidar_num_ <= 0)
        {
            ROS_ERROR("Invalid lidar_num_ value: %d. It must be greater than 0.", lidar_num_);
            ros::shutdown();
            return;
        }

		nh_.getParam("lidar_num", lidar_num_);
        transforms_.resize(lidar_num_);
        clouds_.resize(lidar_num_);

        // 初始化dynamic reconfigure服务器
        dynamic_reconfigure::Server<lidar_fusion_dynamic::LidarFusionConfigConfig>::CallbackType f;
        f = boost::bind(&LidarFusion::configCallback, this, _1, _2);
        server_.setCallback(f);

        // 初始化发布器
        fusion_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(fusion_topic_, 10);

        // 订阅各个雷达的点云数据
        for (int i = 1; i <= lidar_num_; ++i)
        {
            std::string lidar_topic;
            nh_.getParam("lidar_topic" + std::to_string(i), lidar_topic);
			std::cout<<"lidar_topic:"<< lidar_topic <<std::endl;
            subs_.emplace_back(nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, boost::bind(&LidarFusion::lidarCallback, this, _1, i - 1)));
        }
    }

    void configCallback(lidar_fusion_dynamic::LidarFusionConfigConfig &config, uint32_t level)
    {
        if (lidar_num_ > 0)
        {
            // 更新Lidar1的转换矩阵
            Eigen::Affine3d transform1 = Eigen::Affine3d::Identity();
            transform1.translation() << config.lidar1_x, config.lidar1_y, config.lidar1_z;
            transform1.rotate(Eigen::AngleAxisd(config.lidar1_roll, Eigen::Vector3d::UnitX()));
            transform1.rotate(Eigen::AngleAxisd(config.lidar1_pitch, Eigen::Vector3d::UnitY()));
            transform1.rotate(Eigen::AngleAxisd(config.lidar1_yaw, Eigen::Vector3d::UnitZ()));
            transforms_[0] = transform1;
        }

        if (lidar_num_ > 1)
        {
            // 更新Lidar2的转换矩阵
            Eigen::Affine3d transform2 = Eigen::Affine3d::Identity();
            transform2.translation() << config.lidar2_x, config.lidar2_y, config.lidar2_z;
            transform2.rotate(Eigen::AngleAxisd(config.lidar2_roll, Eigen::Vector3d::UnitX()));
            transform2.rotate(Eigen::AngleAxisd(config.lidar2_pitch, Eigen::Vector3d::UnitY()));
            transform2.rotate(Eigen::AngleAxisd(config.lidar2_yaw, Eigen::Vector3d::UnitZ()));
            transforms_[1] = transform2;
        }

		// 如果还有的话
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher fusion_pub_;
    int lidar_num_;
    std::vector<Eigen::Affine3d> transforms_;
    dynamic_reconfigure::Server<lidar_fusion_dynamic::LidarFusionConfigConfig> server_;
    std::string fusion_topic_ = "fusion_lidar";
    std::string fusion_frame_ = "lidar_link";

    std::vector<ros::Subscriber> subs_;
    std::vector<sensor_msgs::PointCloud2> clouds_;  // 使用 sensor_msgs::PointCloud2 直接存储点云

    // 处理每一个雷达的数据
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int lidar_index)
    {
        // 变换点云数据
        sensor_msgs::PointCloud2 transformed_cloud;
        Eigen::Affine3f transform = transforms_[lidar_index].cast<float>();
        pcl_ros::transformPointCloud(transform.matrix(), *msg, transformed_cloud);

        // 存储变换后的点云
        clouds_[lidar_index] = transformed_cloud;

        // 检查是否所有雷达数据都已经到达
        if (std::all_of(clouds_.begin(), clouds_.end(), [](const sensor_msgs::PointCloud2& c) { return c.data.size() > 0; }))
        {
            // 合并所有点云
            sensor_msgs::PointCloud2 fused_cloud = clouds_[0];
            for (size_t i = 1; i < clouds_.size(); ++i)
            {
                pcl::concatenatePointCloud(fused_cloud, clouds_[i], fused_cloud);
            }

            // 发布最终的融合点云
            fused_cloud.header.frame_id = fusion_frame_;
            fusion_pub_.publish(fused_cloud);

            // 清除点云数据以便处理新的数据
            clouds_.clear();
            clouds_.resize(lidar_num_);
        }
    }
};
