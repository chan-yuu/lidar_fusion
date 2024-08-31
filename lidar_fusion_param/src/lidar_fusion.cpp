/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-08-30 15:54:00
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-08-30 15:56:00
 * @FilePath: /undefined/home/cyun/learn_ws/lidar_fusion/lidar_fusion_param/src/lidar_fusion.cpp
 * @Description: lidar fusion use param
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <lidar_fusion_param/lidar_fusion.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_fusion_node");
	LidarFusion lidarFusion;
	ros::spin();
	return 0;
}

