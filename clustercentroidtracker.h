#pragma once
#include <vector>
#include <map>
#include <string>
#include <Eigen/Geometry>
#include <pcl/common/common_headers.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;

class CentroidTracker
{
private:
	int nextObjectID_ = 0;
	int maxDisappeared_;	
public:
	std::map<int, int> disappeared;
	std::map<int, Eigen::Vector3f> objects;
	std::map<int, Eigen::Vector3f> direction;
	std::map<int, XYZcloudPtr> clusters;
public:
	CentroidTracker();
	CentroidTracker(int maxDisappeared);
	void Update(std::vector<XYZcloudPtr> clusters);
	void PrintObjects();
private:
	void UpdateDisappeared(int objectID);
	void Register(XYZcloudPtr cluster, Eigen::Vector3f &centroid);
	void RegisterWithDirection(XYZcloudPtr cluster,
		Eigen::Vector3f& centroid, Eigen::Vector3f& dir);
	void Deregister(int objectID);
	Eigen::MatrixXf GetDistanceMatrix(std::vector<Eigen::Vector3f>& centroids);
};
