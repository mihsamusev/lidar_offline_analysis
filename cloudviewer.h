#pragma once
// native c++ includes
#include <string>
#include <vector>
#include <tuple>

// project includes
#include "cloudframes.h"

// 3rd party includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <clustercentroidtracker.h>

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
	CloudFramesPtr frames_;
	int nFrames_ = 0;
	int currentFrameId_ = 0;
	XYZcloudPtr currentCloudPtr_ = nullptr;

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
	CentroidTracker tracker_;
	bool show_floor_ = false;
	bool show_bg_ = false;
	bool show_roi_ = false;
	std::vector<std::string> clusterNames_;
	std::vector<std::string> bbNames_;
	std::vector<std::tuple<float, float, float>> colorscheme_;
	const std::string& CURRENT_CLOUD_LABEL = "current";
	VizPtr viewer_;

	void DrawBoundingBox(XYZcloudPtr cloud, char* name);
	void RemoveClusters();
	void RemoveBoxes();
	void RenderFrames();
	void UpdateViewer();
	void RenderTracks(CentroidTracker &tracker);
	void RemoveTracks(CentroidTracker &tracker);
	void RenderClusters(std::vector<pcl::PointIndices>& clusterIdx);
	void ProcessNextFrame();
	void ProcessPrevFrame();
	void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud);
	void InitColorScheme(int count);
};