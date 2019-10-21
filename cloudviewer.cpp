// native c++ includes
#include <string>
#include <iostream>
#include <vector>

// project includes
#include <cloudframes.h>
#include <cloudviewer.h>

// 3rd party inlcudes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> // temporary, just for BG
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

// aliases
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr XYZcloudCPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

typedef pcl::visualization::PCLVisualizer::Ptr VizPtr;
typedef pcl::visualization::PCLVisualizer Viz;

// constructor
CloudViewer::CloudViewer()
{
	VizPtr v(new Viz("Cloud Viewer"));
	this->viewer = v;
}

// wrappers for PCLVizualizer
bool CloudViewer::wasStopped()
{
	return this->viewer->wasStopped();
}

void CloudViewer::spinOnce()
{
	this->viewer->spinOnce();
}

void CloudViewer::spin()
{
	this->viewer->spin();
}

// initialize viewer
void CloudViewer::initView()
{
	this->viewer->setBackgroundColor(0, 0, 0);
	this->viewer->addCoordinateSystem(1.0); // scale = 1.0
	this->viewer->initCameraParameters();

	//this->viewer->addPointCloud<pcl::PointXYZ>(this->currentCloudPtr, this->CURRENT_CLOUD_LABEL);
	//this->viewer->setPointCloudRenderingProperties(
		//pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, this->CURRENT_CLOUD_LABEL);
	this->viewer->setRepresentationToWireframeForAllActors();

	void* cookie = nullptr; // all extra user data is send as void pointer (PCL)
	this->viewer->registerKeyboardCallback(&CloudViewer::keyboardEventOccurred, *this, cookie);
}

void CloudViewer::initView(const std::string& camfile)
{
	this->initView();
	this->viewer->loadCameraParameters(camfile);
}

// 
bool CloudViewer::setFrames(CloudFramesPtr f)
{

	if(f->clouds.size() == 0)
	{
		std::cout << "Empty instance of CloudFrames, " <<
			"cloud frames are not set" << std::endl;
		return false;
	}
	
	this->frames = f;
	this->nFrames = f->clouds.size();
	this->currentCloudPtr = this->frames->clouds[this->currentFrameId];
	return true;
}

// put next frame for rendering by viewer
void CloudViewer::loadNextFrame()
{
	int frame = this->currentFrameId;

	// test is frame is not over the bounds
	if (++frame < this->nFrames)
	{
		// update id and pointer
		this->currentFrameId++;
		this->currentCloudPtr = this->frames->clouds[this->currentFrameId]; 
			
		// experimental
		std::vector<pcl::PointIndices> clusterIdx =
			frames->clusterCloud(this->currentCloudPtr, 0.5);
		//this->viewer->removeAllPointClouds();
		renderClusters(clusterIdx);
	}
}

// put prev frame for rendering by viewer
void CloudViewer::loadPrevFrame()
{
	int frame = this->currentFrameId;

	// test is frame is not over the bounds
	if (--frame >= 0)
	{
		// update id and pointer
		this->currentFrameId--;
		this->currentCloudPtr = this->frames->clouds[this->currentFrameId];
		//renderFrames();

		// experimental
		std::vector<pcl::PointIndices> clusterIdx =
			frames->clusterCloud(currentCloudPtr, 1);
		//this->viewer->removeAllPointClouds();
		renderClusters(clusterIdx);
	}
}

void CloudViewer::drawBoundingBox(XYZcloudPtr cloud, char* name)
{
	// Compute centroid and covariance matrix
	Eigen::Vector4f pcaCentroid;
	Eigen::Matrix3f covariance;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

	// get eigenvectors from the covariance matrix,
	// ensure they are forming a right handed coordinate system
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> 
		eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	// Prepare transformation matrix where principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.0f * 
		(projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	
	// transform cloud
	XYZcloudPtr cloudPointsProjected(new XYZcloud);
	pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
	
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f * 
		(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
	const Eigen::Vector3f bboxTransform = 
		eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// create cube
	this->viewer->addCube(bboxTransform, bboxQuaternion,
		maxPoint.x - minPoint.x,
		maxPoint.y - minPoint.y,
		maxPoint.z - minPoint.z, name);
}

void CloudViewer::renderFrames()
{
	this->viewer->removePointCloud(this->CURRENT_CLOUD_LABEL);
	this->viewer->addPointCloud(this->currentCloudPtr, this->CURRENT_CLOUD_LABEL);
	this->viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, this->CURRENT_CLOUD_LABEL);
}

void CloudViewer::renderClusters(std::vector<pcl::PointIndices>& clusterIdx)
{
	// remove clusters
	for (size_t i = 0; i < clusterNames.size(); i++)
	{
		viewer->removePointCloud(clusterNames[i]);
	}
	clusterNames.clear();

	// add clusters
	int n = 0;
	for (auto it = clusterIdx.begin(); it != clusterIdx.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		cluster->width = it->indices.size();
		cluster->height = 1;
		cluster->is_dense = true;
		cluster->points.reserve(cluster->width);
		for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cluster->points.push_back(this->currentCloudPtr->points[*pit]);
		}

		char cloudname[10];
		char boxname[10];
		sprintf(cloudname, "c_%03d", n);
		sprintf(boxname, "b_%03d", n);
		n++;
		clusterNames.push_back(cloudname);

		this->viewer->addPointCloud<pcl::PointXYZ>(cluster, cloudname);
		float r = rand() / (RAND_MAX + 1.0f);
		float g = rand() / (RAND_MAX + 1.0f);
		float b = rand() / (RAND_MAX + 1.0f);
		this->viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudname);
		this->viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloudname);

	}
}

// keys put next and prev frames to viewer for visualization
void CloudViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud)
{
	if (event.getKeySym() == "Right" && event.keyDown()) // proceed frame backwards
	{
		this->loadNextFrame();
	}
	else if (event.getKeySym() == "Left" && event.keyDown()) // proceed frame forwards
	{
		this->loadPrevFrame();
	}
	else if (event.getKeySym() == "n" && event.keyDown()) // turn the floor on and off
	{
		if (this->frames->floor->size() != 0)
		{
			if (this->show_floor)
			{
				this->viewer->removePointCloud("floor");
				this->show_floor = false;
			}
			else
			{
				this->viewer->addPointCloud<pcl::PointXYZ>(this->frames->floor, "floor");
				this->viewer->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "floor");
				this->viewer->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "floor");
				this->show_floor = true;
			}
		}
	}
	else if (event.getKeySym() == "b" && event.keyDown()) // turn the background on and off
	{
		if (this->frames->background->size() != 0)
		{
			if (this->show_bg)
			{
				this->viewer->removePointCloud("background");
				this->show_bg = false;
			}
			else
			{
				this->viewer->addPointCloud<pcl::PointXYZ>(this->frames->background, "background");
				this->viewer->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "background");
				this->viewer->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 1, "background");
				this->show_bg = true;
			}
		}
	}
	else if (event.getKeySym() == "m" && event.keyDown()) // turn the roi
	{
		if (this->frames->roi->size() != 0)
		{
			if (this->show_roi)
			{
				this->viewer->removeShape("roi");
				this->show_roi = false;
			}
			else
			{
				this->viewer->addPolygon<pcl::PointXYZ>(this->frames->roi, "roi");
				this->viewer->setShapeRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "roi");
				this->show_roi = true;
			}
		}
	}
}

