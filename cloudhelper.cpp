// native C++ includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// original includes
#include "cloudhelper.h"

// 3rd party includes
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

// aliases
namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

// PUBLIC METHODS

bool CloudHelper::read_all_clouds(const std::string& path)
{
	// count number of point cloud files
	int nFiles = 0;
	for (fs::directory_entry& cloudFile : fs::directory_iterator(path))
	{
		nFiles++;
	}
	std::cout << nFiles << std::endl;

	// preallocate vector
	this->clouds.reserve(nFiles);

	// read file and put it into vector
	
	for (fs::directory_entry& cloudFile : fs::directory_iterator(path))
	{
		XYZcloudPtr pc(new XYZcloud);
		if (pcl::io::loadPCDFile(cloudFile.path().string(), *pc) == -1)
		{
			PCL_ERROR("Couldn't read file");
			return false;
		}
		this->clouds.push_back(pc);
	}
}


// PRIVATE METHODS

Eigen::Quaterniond CloudHelper::getQuaterniond(float nx, float ny, float nz, float a)
{
	float w = cos(a / 2);
	float x = sin(a / 2) * nx;
	float y = sin(a / 2) * ny;
	float z = sin(a / 2) * nz;
	Eigen::Quaterniond quat(w, x, y, z);
	return quat;
}


