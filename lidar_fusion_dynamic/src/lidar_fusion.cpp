/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-08-30 15:54:00
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-08-30 16:57:22
 * @FilePath: /undefined/home/cyun/learn_ws/lidar_fusion/src/lidar_fusion_dynamic/src/lidar_fusion.cpp
 * @Description: lidar fusion use param
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <lidar_fusion_dynamic/lidar_fusion.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_fusion_node");
	LidarFusion lidarFusion;
	ros::spin();
	return 0;
}

