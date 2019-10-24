// Native c++ includes
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <stdlib.h>

// original includes
#include <cloudframes.h>
#include <cloudviewer.h>

// 3rd party includes
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>

// aliases
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr XYZcloudCPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> XYZcloud;

typedef pcl::visualization::PCLVisualizer::Ptr VizPtr;
typedef pcl::visualization::PCLVisualizer Viz;

typedef std::unique_ptr<CloudViewer> CloudViewerPtr;
typedef std::shared_ptr<CloudFrames> CloudFramesPtr;

// global scope
const std::string PATH = 
	"C:\\Users\\msa\\Documents\\cpp\\pcl_analysis\\data";

// functions
void debugInfo(const std::string type, const std::string message)
{
	std::cout << type << " " << message << std::endl;
}

// main function
int 
main(int argc, char** argv)
{
	// some thats that are later going to be diefined in .ini file
	const std::string CLOUDS_PATH = 
		"C:\\Users\\msa\\Documents\\cpp\\pcl_analysis\\data\\framesdata";
	const std::string BG_PATH =
		"C:/Users/msa/Documents/cpp/pcl_analysis/data/background.pcd";
	const std::string BB_PATH =
		"C:/Users/msa/Documents/cpp/pcl_analysis/data/bb_after_transform.pcd";
	const std::string CAM_PATH =
		"C:/Users/msa/Documents/cpp/pcl_analysis/data/camparam.cam";
	// Initialize CloudFrames class

	debugInfo("[INFO]","Initializing CloudFrames");
	CloudFramesPtr ch = std::make_shared<CloudFrames>();

	debugInfo("[INFO]", "Reading background from " + BG_PATH);
	ch->readBackground(BG_PATH);	// in lidar CC

	debugInfo("[INFO]", "Reading all clouds from " + CLOUDS_PATH);
	ch->readClouds(CLOUDS_PATH); // in lidar CC
	
	debugInfo("[INFO]", "Reading region of interest from " + BB_PATH);
	ch->readROI(BB_PATH); // in road plane CC, remember that!

	// Detect floor with plane fitting RANSAC
	// The coordinate system is now based on the floor
	// match road plane normal and z axes
	debugInfo("[INFO]", "Estimating road plane and transforming data to road alligned CC");
	ch->estimateFloor();
	ch->transformToMatchFloor();

	// crop everything by ROI and remove background
	debugInfo("[INFO]", "Filtering by ROI and subtracting background");
	ch->cropByROI();
	ch->subtractBackground();

	debugInfo("[INFO]", "Initializing CloudViewer");
	CloudViewerPtr viewer_ = std::make_unique<CloudViewer>();
	if (viewer_->setFrames(ch))
	{
		viewer_->initView(CAM_PATH);
	}

	debugInfo("[INFO]", "Running visualization");
	while(!viewer_->wasStopped())
	{
		viewer_->spin();
	}
}