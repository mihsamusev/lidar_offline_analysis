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
	viewer_ = v;
}

// wrappers for PCLVizualizer
bool CloudViewer::wasStopped()
{
	return viewer_->wasStopped();
}

void CloudViewer::spinOnce()
{
	viewer_->spinOnce();
}

void CloudViewer::spin()
{
	viewer_->spin();
}

// initialize viewer_
void CloudViewer::initView()
{
	viewer_->setBackgroundColor(0, 0, 0);
	viewer_->addCoordinateSystem(1.0); // scale = 1.0
	viewer_->initCameraParameters();

	//viewer_->addPointCloud<pcl::PointXYZ>(currentCloudPtr_, CURRENT_CLOUD_LABEL);
	//viewer_->setPointCloudRenderingProperties(
		//pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, CURRENT_CLOUD_LABEL);
	viewer_->setRepresentationToWireframeForAllActors();

	void* cookie = nullptr; // all extra user data is send as void pointer (PCL)
	viewer_->registerKeyboardCallback(&CloudViewer::KeyboardEventOccurred, *this, cookie);
}

void CloudViewer::initView(const std::string& camfile)
{
	initView();
	viewer_->loadCameraParameters(camfile);
}

// 
bool CloudViewer::setFrames(CloudFramesPtr f)
{

	if(f->clouds.size() == 0)
	{
		std::cout << "Empty instance of CloudFrames, " <<
			"cloud frames_ are not set" << std::endl;
		return false;
	}
	
	frames_ = f;
	nFrames_ = f->clouds.size();
	currentCloudPtr_ = frames_->clouds[currentFrameId_];
	return true;
}

// put next frame for rendering by viewer_
void CloudViewer::LoadNextFrame()
{
	int frame = currentFrameId_;

	// test is frame is not over the bounds
	if (++frame < nFrames_)
	{
		// update id and pointer
		currentFrameId_++;
		currentCloudPtr_ = frames_->clouds[currentFrameId_]; 
			
		// experimental
		std::vector<pcl::PointIndices> clusterIdx =
			frames_->clusterCloud(currentCloudPtr_, 1);
		//viewer_->removeAllPointClouds();

		RemoveClusters();
		RemoveBoxes();
		RenderClusters(clusterIdx);
	}
}

// put prev frame for rendering by viewer_
void CloudViewer::LoadPrevFrame()
{
	int frame = currentFrameId_;

	// test is frame is not over the bounds
	if (--frame >= 0)
	{
		// update id and pointer
		currentFrameId_--;
		currentCloudPtr_ = frames_->clouds[currentFrameId_];
		//RenderFrames();

		// experimental
		std::vector<pcl::PointIndices> clusterIdx =
			frames_->clusterCloud(currentCloudPtr_, 1);
		//viewer_->removeAllPointClouds();

		RemoveClusters();
		RemoveBoxes();
		RenderClusters(clusterIdx);
	}
}

void CloudViewer::DrawBoundingBox(XYZcloudPtr cloud, char* name)
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
	viewer_->addCube(bboxTransform, bboxQuaternion,
		maxPoint.x - minPoint.x,
		maxPoint.y - minPoint.y,
		maxPoint.z - minPoint.z, name);
	viewer_->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
}

void CloudViewer::RenderFrames()
{
	viewer_->removePointCloud(CURRENT_CLOUD_LABEL);
	viewer_->addPointCloud(currentCloudPtr_, CURRENT_CLOUD_LABEL);
	viewer_->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, CURRENT_CLOUD_LABEL);
}

void CloudViewer::RemoveClusters()
{
	for (size_t i = 0; i < clusterNames_.size(); i++)
	{
		viewer_->removePointCloud(clusterNames_[i]);
	}
	clusterNames_.clear();
}

void CloudViewer::RemoveBoxes()
{
	for (size_t i = 0; i < bbNames_.size(); i++)
	{
		viewer_->removeShape(bbNames_[i]);
	}
	bbNames_.clear();
}

void CloudViewer::RenderClusters(std::vector<pcl::PointIndices>& clusterIdx)
{
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
			cluster->points.push_back(currentCloudPtr_->points[*pit]);
		}

		// take care of naming
		char cloudname[10];
		char boxname[10];
		sprintf(cloudname, "c_%03d", n);
		sprintf(boxname, "b_%03d", n);
		n++;
		clusterNames_.push_back(cloudname);
		bbNames_.push_back(boxname);

		// draw cluster
		viewer_->addPointCloud<pcl::PointXYZ>(cluster, cloudname);
		float r = rand() / (RAND_MAX + 1.0f);
		float g = rand() / (RAND_MAX + 1.0f);
		float b = rand() / (RAND_MAX + 1.0f);
		viewer_->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudname);
		viewer_->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloudname);

		if (cluster->points.size() > 20)
		{
			//DrawBoundingBox(cluster, boxname);
			MinAreaBoundingBox myBox = frames_->GetMinAreaBox(cluster);
			viewer_->addCube(myBox.translation, myBox.quaternion, 
				myBox.xDim, myBox.yDim, myBox.zDim, boxname);
			viewer_->setShapeRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
				pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, boxname);
		}
	}
}

// keys put next and prev frames_ to viewer_ for visualization
void CloudViewer::KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud)
{
	if (event.getKeySym() == "Right" && event.keyDown()) // proceed frame backwards
	{
		LoadNextFrame();
	}
	else if (event.getKeySym() == "Left" && event.keyDown()) // proceed frame forwards
	{
		LoadPrevFrame();
	}
	else if (event.getKeySym() == "n" && event.keyDown()) // turn the floor on and off
	{
		if (frames_->floor->size() != 0)
		{
			if (show_floor_)
			{
				viewer_->removePointCloud("floor");
				show_floor_ = false;
			}
			else
			{
				viewer_->addPointCloud<pcl::PointXYZ>(frames_->floor, "floor");
				viewer_->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "floor");
				viewer_->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "floor");
				show_floor_ = true;
			}
		}
	}
	else if (event.getKeySym() == "b" && event.keyDown()) // turn the background on and off
	{
		if (frames_->background->size() != 0)
		{
			if (show_bg_)
			{
				viewer_->removePointCloud("background");
				show_bg_ = false;
			}
			else
			{
				viewer_->addPointCloud<pcl::PointXYZ>(frames_->background, "background");
				viewer_->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "background");
				viewer_->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 1, "background");
				show_bg_ = true;
			}
		}
	}
	else if (event.getKeySym() == "m" && event.keyDown()) // turn the roi
	{
		if (frames_->roi->size() != 0)
		{
			if (show_roi_)
			{
				viewer_->removeShape("roi");
				show_roi_ = false;
			}
			else
			{
				viewer_->addPolygon<pcl::PointXYZ>(frames_->roi, "roi");
				viewer_->setShapeRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "roi");
				show_roi_ = true;
			}
		}
	}
	else if (event.getKeySym() == "v" && event.keyDown())
	{
		std::vector<pcl::PointIndices> clusterIdx =
			frames_->clusterCloud(currentCloudPtr_, 1);

		int maxClusterPos = 0;
		int maxClusterSize = 0;

		for (size_t i = 0; i < clusterIdx.size(); i++)
		{
			if (clusterIdx[i].indices.size() >= maxClusterSize)
			{
				maxClusterSize = clusterIdx[i].indices.size();
				maxClusterPos = i;
			}
		}

		pcl::PointIndices maxClusterIdx = clusterIdx[maxClusterPos];
		pcl::PointCloud<pcl::PointXYZ>::Ptr maxCluster(new pcl::PointCloud<pcl::PointXYZ>);
		maxCluster->width = maxClusterIdx.indices.size();
		maxCluster->height = 1;
		maxCluster->is_dense = true;
		maxCluster->points.reserve(maxCluster->width);
		for (auto pit = maxClusterIdx.indices.begin(); pit != maxClusterIdx.indices.end(); ++pit)
		{
			maxCluster->points.push_back(currentCloudPtr_->points[*pit]);
		}

		std::string name = "largest_cloud" + std::to_string(currentFrameId_) + ".pcd";
		int result = pcl::io::savePCDFile(name, *maxCluster);

		std::cout << "saving file: " << result << std::endl;
	}
}

