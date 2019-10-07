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

class CloudFrames
{
public:
	std::vector<XYZcloudPtr> clouds;
	XYZcloudPtr floor;
	XYZcloudPtr background;

	bool read_all_clouds(const std::string& path);
	void rotateAll(float nx, float ny, float nz, float a);
	void fitPlane(XYZcloudPtr cloud_in, XYZcloudPtr cloud_out, double tol);
	void subtractBG(XYZcloudPtr bg_cloud, XYZcloudPtr cloud_in, XYZcloudPtr cloud_out,
		double distTol, double stdCoeff);
	void segmentBG(XYZcloudPtr bg_cloud, XYZcloudPtr cloud_in, XYZcloudPtr cloud_out,
		double distTol, double stdCoeff);
	void segmentBG_all(const std::string& bg_path);
private:
	Eigen::Quaterniond getQuaterniond(float nx, float ny, float nz, float a);
	void rotateCloud(XYZcloudPtr cloudPtr, Eigen::Quaterniond& quat);
};
