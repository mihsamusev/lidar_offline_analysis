// native C++ includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

// original includes
#include "cloudframes.h"

// 3rd party includes
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/segment_differences.h>

#include <pcl/segmentation//extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>

// aliases
namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

// PUBLIC METHODS

bool CloudFrames::readBackground(const std::string& path)
{
	// error checking here maybe
	XYZcloudPtr bg(new XYZcloud);
	pcl::io::loadPCDFile(path, *bg);
	this->background = bg;
	return true;
}

bool CloudFrames::readROI(const std::string& path)
{
	// error checking here maybe
	XYZcloudPtr roi(new XYZcloud);
	pcl::io::loadPCDFile(path, *roi);
	this->roi = roi;
	return true;
}

bool CloudFrames::readClouds(const std::string& path)
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
		XYZcloudPtr cloud(new XYZcloud);
		if (pcl::io::loadPCDFile(cloudFile.path().string(), *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file");
			return false;
		}
		if (this->floorAlignedCC)
		{
			transformToFloorAlignedCC(cloud);
		}
		this->clouds.push_back(cloud);
	}

	return true;
}



bool CloudFrames::estimateFloor()
{
	XYZcloudPtr fl(new XYZcloud);

	double tol = 0.2;

	// get floor from background if possible, else try
	// from the very first cloud
	if (this->background->points.size() != 0)
	{
		fitPlane(this->background, fl, this->plane_coeff, tol);
		this->floor = fl;
		return true;
	}
	else if(this->clouds.size() != 0)
	{
		fitPlane(this->clouds[0], fl, this->plane_coeff, tol);
		this->floor = fl;
		return true;
	}
	else
	{
		std::cout << "[ERROR] missing background cloud or at least one frame cloud to perform plane fitting\n";
		return false;
	}
}

bool CloudFrames::transformToMatchFloor()
{
	this->floorAlignedCC = true;

	if (!transformToFloorAlignedCC(this->floor))
	{
		std::cout << "[ERROR] floor has not been estimated yet and needs to be initialized first\n";
		return false;
	}

	if (!transformToFloorAlignedCC(this->background))
	{
		std::cout << "[ERROR] floor has not been estimated yet and needs to be initialized first\n";
		return false;
	}

	for (size_t i = 0; i < this->clouds.size(); i++)
	{
		transformToFloorAlignedCC(this->clouds[i]);
	}

	return true;
}

bool CloudFrames::cropByROI()
{
	if (this->roi->points.size() != 0)
	{
		// incude error checking at some point
		cropByXYPoly(this->floor, this->roi);
		cropByXYPoly(this->background, this->roi);

		for (size_t i = 0; i < this->clouds.size(); i++)
		{
			cropByXYPoly(this->clouds[i], this->roi);
		}

		return true;
	}
}

bool CloudFrames::subtractBackground()
{
	if (this->background->points.size() != 0)
	{
		for (size_t i = 0; i < this->clouds.size(); i++)
		{
			XYZcloudPtr nobg(new XYZcloud);
			subtractBG(this->background, this->clouds[i], nobg, 0.1, 1);
			this->clouds[i] = nobg;
		}
		return true;
	}
	else
	{
		std::cout << "[ERROR] missing background cloud to perform substraction\n";
		return false;
	}
}



std::vector<pcl::PointIndices> 
CloudFrames::clusterCloud(XYZcloudPtr cloud, float tol)
{
	int minPts = cloud->points.size() / 1000; // 0.1% pts
	int maxPts = cloud->points.size(); // all pts

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tol);
	ec.setMinClusterSize(minPts);
	ec.setMaxClusterSize(maxPts);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr 
		tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	return cluster_indices;
}

// PRIVATE METHODS

void CloudFrames::cropByXYPoly(XYZcloudPtr cloud_in, XYZcloudPtr poly)
{
	// allocate vector of inliers
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	int cloudPts = cloud_in->points.size();
	inliers->indices.reserve(cloudPts / 4);

	// check every point is within a polygon
	for (size_t i = 0; i < cloudPts; i++)
	{
		if (pcl::isPointIn2DPolygon(cloud_in->points[i], *poly))
		{
			inliers->indices.push_back(i);
		}
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(inliers);
	extract.filter(*cloud_in);
}

void CloudFrames::fitPlane(XYZcloudPtr cloud_in, XYZcloudPtr cloud_out, 
	Eigen::VectorXf &plane_coeff, double tol=0.1)
{
	// indices of points giving closest match to ransac plane model
	std::vector<int>inliers;

	// created RandomSampleConsensus object and compute the model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
		modelp(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_in));
	pcl::RandomSampleConsensus<pcl::PointXYZ>ransac(modelp);
	ransac.setDistanceThreshold(tol);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// obtain plane coefficients (A,B,C,D) for Ax + By + Cz + D = 0
	// where (A,B,C) is the unit normal vector
	ransac.getModelCoefficients(plane_coeff);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *cloud_out);
}

Eigen::Quaterniond CloudFrames::getQuaterniond(float nx, float ny, float nz, float a)
{
	float w = cos(a / 2);
	float x = sin(a / 2) * nx;
	float y = sin(a / 2) * ny;
	float z = sin(a / 2) * nz;
	Eigen::Quaterniond quat(w, x, y, z);
	return quat;
}

Eigen::Quaterniond CloudFrames::getQuaterniond(Eigen::Vector3f normal, Eigen::Vector3f axis)
{
	// make sure vectors have unit length
	normal.normalize();
	axis.normalize();
	
	// find angle between vectors
	float theta = acos(normal.dot(axis));
	
	// find rotation axis and ensure its unit length
	Eigen::Vector3f rotAxis = normal.cross(axis);
	rotAxis.normalize();
	Eigen::Quaterniond q = getQuaterniond(rotAxis(0), rotAxis(1), rotAxis(2), theta);
	return q;
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

bool CloudFrames::transformToFloorAlignedCC(XYZcloudPtr cloudPtr)
{
	if (cloudPtr->points.size() != 0)
	{
		// Define transform based on the plane coefficients
		// calculate at initFloor => fitPlane
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		double D = this->plane_coeff(3);
		double dx = D * this->plane_coeff(0);
		double dy = D * this->plane_coeff(1);
		double dz = D * this->plane_coeff(2);

		// Define a translation of D units along normal N = (dx, dy, dz)
		transform.translation() << dx, dy, dz;

		// Executing the transformation for floor
		pcl::transformPointCloud(*cloudPtr, *cloudPtr, transform);
		
		// calculate quaternion and rotate the point cloud
		Eigen::Vector3f normal(this->plane_coeff(0), this->plane_coeff(1), this->plane_coeff(2));
		Eigen::Vector3f zaxis = Eigen::Vector3f::UnitZ();
		Eigen::Quaterniond q = getQuaterniond(normal, zaxis);
		rotateCloud(cloudPtr, q);

		return true;
	}
	else
	{
		return false; 
	}
}

void CloudFrames::subtractBG(XYZcloudPtr bg_cloud,
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




