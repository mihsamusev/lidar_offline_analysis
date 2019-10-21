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
#include <pcl/PointIndices.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;

class CloudFrames
{
public:
	std::vector<XYZcloudPtr> clouds;
	XYZcloudPtr floor;
	XYZcloudPtr background;
	XYZcloudPtr roi;

	bool readROI(const std::string& path);
	bool readBackground(const std::string& path);
	bool readClouds(const std::string& path);

	bool estimateFloor();
	bool transformToMatchFloor();

	bool cropByROI();
	bool subtractBackground();

	std::vector<pcl::PointIndices> clusterCloud(XYZcloudPtr cloud, float tol);
private:
	bool floorAlignedCC = false;
	Eigen::VectorXf plane_coeff;

	void fitPlane(XYZcloudPtr cloud_in, XYZcloudPtr cloud_out, 
		Eigen::VectorXf &plane_coeff, double tol);
	Eigen::Quaterniond getQuaterniond(float nx, float ny, float nz, float a);
	Eigen::Quaterniond getQuaterniond(Eigen::Vector3f v1, Eigen::Vector3f v2);
	void rotateCloud(XYZcloudPtr cloudPtr, Eigen::Quaterniond& quat);
	bool transformToFloorAlignedCC(XYZcloudPtr cloudPtr);
	void cropByXYPoly(XYZcloudPtr cloud_in, XYZcloudPtr poly);

	void subtractBG(XYZcloudPtr bg_cloud, XYZcloudPtr cloud_in, XYZcloudPtr cloud_out,
		double distTol, double stdCoeff);
};
