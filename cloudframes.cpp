// native C++ includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// original includes
#include "cloudframes.h"

// 3rd party includes
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/segment_differences.h>

// aliases
namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

// PUBLIC METHODS

bool CloudFrames::read_all_clouds(const std::string& path)
{
	// check if path exists and is directory
	if(!fs::is_directory(path))
	{
		PCL_ERROR("The path doesnt exist or its not a directory");
		return false;
	}

	// count number of point cloud files
	int nFiles = 0;
	auto d = fs::directory_iterator(path);
	for (fs::directory_entry& cloudFile : fs::directory_iterator(path))
	{
		nFiles++;
	}

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

	return true;
}


// PRIVATE METHODS

Eigen::Quaterniond CloudFrames::getQuaterniond(float nx, float ny, float nz, float a)
{
	float w = cos(a / 2);
	float x = sin(a / 2) * nx;
	float y = sin(a / 2) * ny;
	float z = sin(a / 2) * nz;
	Eigen::Quaterniond quat(w, x, y, z);
	return quat;
}

void CloudFrames::rotateCloud(XYZcloudPtr cloudPtr, Eigen::Quaterniond& quat)
{
	for (size_t i = 0; i < cloudPtr->points.size(); i++)
	{
		Eigen::Vector3d pnt(cloudPtr->points[i].x,
							cloudPtr->points[i].y,
							cloudPtr->points[i].z);
		Eigen::Vector3d pntRot = quat * pnt;

		cloudPtr->points[i].x = pntRot.coeff(0);
		cloudPtr->points[i].y = pntRot.coeff(1);
		cloudPtr->points[i].z = pntRot.coeff(2);
	}
}

void CloudFrames::rotateAll(float nx, float ny, float nz, float a)
{
	Eigen::Quaterniond q = getQuaterniond(nx, ny, nz, a);
	
	for (size_t i = 0; i < this->clouds.size(); i++)
	{
		rotateCloud(this->clouds[i], q); // shoud i do double pointer here?
	}
}

void CloudFrames::fitPlane(XYZcloudPtr cloud_in, XYZcloudPtr cloud_out, double tol=0.1)
{
	std::vector<int>inliers;
	// created RandomSampleConsensus object and compute the model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelp(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_in));
	pcl::RandomSampleConsensus<pcl::PointXYZ>ransac(modelp);
	ransac.setDistanceThreshold(tol);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *cloud_out);
}

void CloudFrames::subtractBG(XYZcloudPtr bg_cloud,
							 XYZcloudPtr cloud_in,
							 XYZcloudPtr cloud_out,
							 double distTol=0.3, double stdCoeff=1)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(bg_cloud);
	kdtree.setEpsilon(0);

	// for each of the points in the current frame
	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		float x1 = cloud_in->points[i].x;
		float y1 = cloud_in->points[i].y;
		float z1 = cloud_in->points[i].z;

		double min_dist = 10000000;

		for (int z = 0; z < bg_cloud->size(); z++)
		{
			float x2 = bg_cloud->points[z].x;
			float y2 = bg_cloud->points[z].y;
			float z2 = bg_cloud->points[z].z;

			// euclidean dinstance between points
			double n_my_dist = sqrt((x2 - x1) * (x2 - x1) + 
									(y2 - y1) * (y2 - y1) + 
									(z2 - z1) * (z2 - z1));

			if (n_my_dist < min_dist)
			{
				min_dist = n_my_dist;
			}
		}

		if (min_dist > distTol)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = cloud_in->points[i].x;
			basic_point.y = cloud_in->points[i].y;
			basic_point.z = cloud_in->points[i].z;
			cloud_out->points.push_back(basic_point);
		}
	}
	

	// remove outliers
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_out);
	sor.setMeanK(5);
	sor.setStddevMulThresh(stdCoeff);
	sor.filter(*cloud_out);
	
}

void CloudFrames::segmentBG(XYZcloudPtr bg_cloud,
							XYZcloudPtr cloud_in,
							XYZcloudPtr cloud_out,
							double distTol=0.1, double stdCoeff=1)
{
	// find difference between bg cloud and a frame
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
	sdiff.setInputCloud(cloud_in);
	sdiff.setTargetCloud(bg_cloud);
	sdiff.setSearchMethod(kdtree);
	sdiff.setDistanceThreshold(distTol);
	sdiff.segment(*cloud_out);

	// filter he difference cloud with statistical outlier removal
	///*
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_out);
	sor.setMeanK(50);
	sor.setStddevMulThresh(stdCoeff);  // sor.setNegative(true); // to observe outlier values
	sor.filter(*cloud_out);//*/
}

void CloudFrames::segmentBG_all(const std::string& bg_path)
{
	XYZcloudPtr bg(new XYZcloud);
	pcl::io::loadPCDFile(bg_path, *bg);

	for (size_t i = 0; i < this->clouds.size(); i++)
	{
		XYZcloudPtr nobg(new XYZcloud);
		segmentBG(bg, this->clouds[i], nobg, 0.1, 1);
		this->clouds[i] = nobg;
	}
}


