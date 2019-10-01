#pragma once
// native C++ includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// 3rd party
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;

class CloudHelper
{
public:
	std::vector<XYZcloudPtr> clouds;
	bool read_all_clouds(const std::string& path);
	void rotate_cloud(XYZcloudPtr cloudPtr, Eigen::Quaterniond& quat);
private:
	Eigen::Quaterniond getQuaterniond(float nx, float ny, float nz, float a);

};
