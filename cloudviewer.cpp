// project includes
#include <cloudviewer.h>

// native c++ includes
#include <string>
#include <iostream>
#include <vector>
#include <tuple>

// 3rd party inlcudes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> // temporary, just for BG
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <cloudframes.h>
#include <clustercentroidtracker.h>

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
	tracker_ = CentroidTracker(5);
	InitColorScheme(255);
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
void CloudViewer::ProcessNextFrame()
{
	// test is frame is not over the bounds
	int frame = currentFrameId_;
	if (++frame < nFrames_) {
		// update id, pointer, calculate clusters for
		// current cloud and render them
		currentFrameId_++;
		currentCloudPtr_ = frames_->clouds[currentFrameId_]; 
		// render veiwer with clusters boxes and ids
		UpdateViewer();
	}
}

// put prev frame for rendering by viewer_
void CloudViewer::ProcessPrevFrame()
{
	// test is frame is not over the bounds
	int frame = currentFrameId_;
	if (--frame >= 0) {
		// update id, pointer, calculate clusters for
		// current cloud and render them
		currentFrameId_--;
		currentCloudPtr_ = frames_->clouds[currentFrameId_];
		// render veiwer with clusters boxes and ids
		UpdateViewer();
	}
}

void CloudViewer::UpdateViewer()
{
	// clean viewer from graphics of previous frame
	RemoveTracks(tracker_);
	RemoveClusters();

	// cluster current cloud
	std::vector<pcl::PointIndices> clusterIdx =
		frames_->clusterCloud(currentCloudPtr_, 1); // magic number

	// Get clouds and cloud centers from indices for tracking
	std::vector<XYZcloudPtr> clusters;
	std::vector<Eigen::Vector3f> clusterCentroids;

	clusters.reserve(clusterIdx.size());
	clusterCentroids.reserve(clusterIdx.size());

	for (int i = 0; i < clusterIdx.size(); i++) {
		// only process large clusters
		if (clusterIdx[i].indices.size() < 20) { // magic number
			continue;
		}

		clusters.emplace_back(new XYZcloud);
		boost::shared_ptr<pcl::PointIndices> clusterIdxPtr =
			boost::make_shared<pcl::PointIndices>(clusterIdx[i]);
		frames_->GetSubcloud(currentCloudPtr_, clusterIdxPtr, clusters[i]);

		Eigen::Vector4f center;
		pcl::compute3DCentroid(*clusters[i], center);
		clusterCentroids.emplace_back(center(0), center(1), center(2));
	}

	// update tracker and print list of objects
	tracker_.Update(clusters);
	tracker_.PrintObjects();

	// visualize trackings
	RenderTracks(tracker_);
	//RenderFrames();

	//std::string name = std::to_string(currentFrameId_) + ".png";
	//viewer_->saveScreenshot(name);
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
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, CURRENT_CLOUD_LABEL);
}

void CloudViewer::RemoveClusters()
{
	for (size_t i = 0; i < clusterNames_.size(); i++)
	{
		viewer_->removePointCloud(clusterNames_[i]);
		viewer_->removeText3D("ID_" + std::to_string(i));
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

void CloudViewer::RenderTracks(CentroidTracker &tracker)
{
	std::map<int, Eigen::Vector3f>::iterator it;
	for (it = tracker.objects.begin(); it != tracker.objects.end(); it++) {
		int objectID = it->first;
		if (tracker.disappeared[objectID] > 0){
			continue;
		}
		std::string text = "ID_" + std::to_string(objectID);
		std::string textTag = "text_" + std::to_string(objectID);
		std::string boxTag = "box_" + std::to_string(objectID);
		std::string clusterTag = "cluster_" + std::to_string(objectID);

		// render label
		pcl::PointXYZ position(it->second(0), it->second(1), it->second(2));
		viewer_->addText3D(text, position, 1, 0, 1, 0, textTag);

		// render bounding box
		MinAreaBoundingBox myBox = frames_->GetMinAreaBox(tracker.clusters[objectID]);
		viewer_->addCube(myBox.translation, myBox.quaternion,
			myBox.xDim, myBox.yDim, myBox.zDim, boxTag);
		viewer_->setShapeRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, boxTag);
		viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
			0, 1, 0, boxTag);

		// render cluster itself	
		viewer_->addPointCloud<pcl::PointXYZ>(tracker.clusters[objectID], clusterTag);
		float r = std::get<0>(colorscheme_[objectID]);
		float g = std::get<1>(colorscheme_[objectID]);
		float b = std::get<2>(colorscheme_[objectID]);
		viewer_->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, clusterTag);
		viewer_->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, clusterTag);	
	}
}

void CloudViewer::RemoveTracks(CentroidTracker &tracker)
{
	std::map<int, Eigen::Vector3f>::iterator it;
	for (it = tracker.objects.begin(); it != tracker.objects.end(); it++) {
		int objectID = it->first;
		std::string textTag = "text_" + std::to_string(objectID);
		std::string boxTag = "box_" + std::to_string(objectID);
		std::string clusterTag = "cluster_" + std::to_string(objectID);
		viewer_->removeText3D(textTag);
		viewer_->removeShape(boxTag);
		viewer_->removePointCloud(clusterTag);
	}
}

void CloudViewer::RenderClusters(std::vector<pcl::PointIndices>& clusterIdx)
{
	// add clusters
	int clusterID = 0;
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
		sprintf(cloudname, "c_%03d", clusterID);
		sprintf(boxname, "b_%03d", clusterID);

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
		/*
		if (cluster->points.size() > 20)
		{
			//DrawBoundingBox(cluster, boxname);
			MinAreaBoundingBox myBox = frames_->GetMinAreaBox(cluster);
			viewer_->addCube(myBox.translation, myBox.quaternion, 
				myBox.xDim, myBox.yDim, myBox.zDim, boxname);
			viewer_->setShapeRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
				pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, boxname);

			std::string textname = "ID_" + std::to_string(clusterID);
			viewer_->addText3D(textname, cluster->points[0], 0.5,
				r, g, b, textname);
			viewer_->setShapeRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, textname);
		}
		*/
		clusterID++;
	}
}

// keys put next and prev frames_ to viewer_ for visualization
void CloudViewer::KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud)
{
	if (event.getKeySym() == "Right" && event.keyDown()) // proceed frame backwards
	{
		ProcessNextFrame();
	}
	else if (event.getKeySym() == "Left" && event.keyDown()) // proceed frame forwards
	{
		ProcessPrevFrame();
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
		// total reset
		//viewer_->removePointCloud(CURRENT_CLOUD_LABEL);
		RemoveTracks(tracker_);
		tracker_ = CentroidTracker(5); // magic number

		currentFrameId_= 0;
		currentCloudPtr_ = frames_->clouds[0];
		UpdateViewer();
	}
}

void CloudViewer::InitColorScheme(int count)
{
	colorscheme_.reserve(count);
	for (size_t i = 0; i < count; i++)
	{
		float r = rand() / (RAND_MAX + 1.0f);
		float g = rand() / (RAND_MAX + 1.0f);
		float b = rand() / (RAND_MAX + 1.0f);
		colorscheme_.emplace_back(r, g, b);
	}
}


