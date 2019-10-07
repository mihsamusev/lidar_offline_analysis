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
const std::string PATH = "C:\\Users\\msa\\Documents\\cpp\\pcl_analysis\\data";
unsigned int pc_id = 19000;

// functions
void 
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

// main function
int 
main(int argc, char** argv)
{
	// Parse command arguments
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	double tol;
	int inputPos = pcl::console::find_argument(argc, argv, "-t");
	if (inputPos >= 0)
	{
		tol = atof(argv[inputPos + 1]);
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}

	std::cout << "Loading point clouds...\n";
	CloudFramesPtr ch = std::make_shared<CloudFrames>();
	const std::string PATH2 = 
		"C:\\Users\\msa\\Documents\\cpp\\pcl_analysis\\data\\testframes";
	const std::string BG_PATH =
		"C:/Users/msa/Documents/cpp/pcl_analysis/data/background.pcd";
	// loud clouds to memory
	ch->read_all_clouds(PATH2);
	ch->segmentBG_all(BG_PATH);

	float a = - 3.1415 / 3.0;
	//ch->rotateAll(0.0, 1.0, 0.0, a);

	std::cout << "Initializing cloud viewer\n";
	CloudViewerPtr viewer = std::make_unique<CloudViewer>();
	if (viewer->setFrames(ch))
	{
		viewer->initView();
	}

	std::cout << "Running visualization\n";
	while(!viewer->wasStopped())
	{
		// rotate current maybe?

		// substract bg
		viewer->spinOnce();
	}
}