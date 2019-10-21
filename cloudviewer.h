#pragma once
// native c++ includes
#include <string>
#include <vector>

// project includes
#include "cloudframes.h"

// 3rd party includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

// aliases
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr XYZcloudCPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

typedef pcl::visualization::PCLVisualizer::Ptr VizPtr;
typedef pcl::visualization::PCLVisualizer Viz;

typedef std::shared_ptr<CloudFrames> CloudFramesPtr;

// declarations
class CloudViewer
{
public:
	CloudFramesPtr frames;
	int nFrames = 0;
	int currentFrameId = 0;
	XYZcloudPtr currentCloudPtr = nullptr;

	// constructors
	CloudViewer();

	// wrappers for PCLVisualizer
	bool wasStopped();
	void spinOnce();
	void spin();
	// 
	bool setFrames(CloudFramesPtr f);
	void initView();
	void initView(const std::string& camfile);
private:
	bool show_floor = false;
	bool show_bg = false;
	bool show_roi = false;
	std::vector<std::string> clusterNames;
	std::vector<std::string> bbNames;

	const std::string& CURRENT_CLOUD_LABEL = "current";
	VizPtr viewer;

	void drawBoundingBox(XYZcloudPtr cloud, char* name);
	void renderFrames();
	void renderClusters(std::vector<pcl::PointIndices>& clusterIdx);
	void loadNextFrame();
	void loadPrevFrame();
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud);
};