// Native c++ includes
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <stdlib.h>

// original includes
#include <cloudhelper.h>

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

void 
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	Viz* viewer = static_cast<Viz*> (viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown() && pc_id < 20499)
	{
		char pc_filename[512];
		sprintf(pc_filename, "\\frames\\frame%d.pcd", ++pc_id);
		std::cout << "RIGHT was pressed => updating the point cloud to " << PATH + pc_filename << std::endl;

		XYZcloudPtr new_pc(new XYZcloud);
		pcl::io::loadPCDFile(PATH + pc_filename, *new_pc);
		viewer->updatePointCloud(new_pc, "cloud1");
	}
	else if (event.getKeySym() == "Left" && event.keyDown() && pc_id > 19000)
	{
		char pc_filename[512];
		sprintf(pc_filename, "\\frames\\frame%d.pcd", --pc_id);
		std::cout << "LEFT was pressed => updating the point cloud to " << PATH + pc_filename << std::endl;

		XYZcloudPtr new_pc(new XYZcloud);
		pcl::io::loadPCDFile(PATH + pc_filename, *new_pc);
		viewer->updatePointCloud(new_pc, "cloud1");
	}

	if (event.getKeySym() == "m" && event.keyDown())
	{
		viewer->saveCameraParameters(PATH + "\\camparam.cam");
		std::cout << "Saved camera parameters" << std::endl;
	}
}

VizPtr 
scroll(XYZcloudCPtr initCloud)
{
	// instance of viewer
	VizPtr viewer(new Viz("3D Viewer"));

	// initial settings
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(initCloud, "cloud1");
	viewer->addCoordinateSystem(1.0); // scale = 1.0
	viewer->initCameraParameters();
	viewer->loadCameraParameters(PATH + "\\camparam.cam");

	// listener, viewer.get() returns raw pointer from boost::shared_ptr and (void*) casts it to void 
	// pointer to provide type independent input
	void* userData = (void*)viewer.get();
	viewer->registerKeyboardCallback(keyboardEventOccurred, userData);
	return (viewer);
}

VizPtr
scrolldouble(XYZcloudCPtr initCloud, XYZcloudCPtr background)
{
	// instance of viewer
	VizPtr viewer(new Viz("3D Viewer"));

	// specify settings common for both viewports
	viewer->initCameraParameters();
	viewer->loadCameraParameters(PATH + "\\camparam.cam");

	// initial settings for viewport 0
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
	viewer->addPointCloud<pcl::PointXYZ>(initCloud, "cloud1", v1);
	viewer->addCoordinateSystem(1.0); // scale = 1.0

	// listener, viewer.get() returns raw pointer from boost::shared_ptr and (void*) casts it to void 
	// pointer to provide type independent input (C style flex)
	void* userData = (void*)viewer.get();
	viewer->registerKeyboardCallback(keyboardEventOccurred, userData);

	// initial settings for viewport 1
	int v2(1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addPointCloud<pcl::PointXYZ>(background, "background", v2);

	return (viewer);
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

	// Initialize start point
	std::cout << "Loading point clouds.\n\n";
	CloudHelper ch;
	ch.read_all_clouds(PATH + "\\frames");
	for (size_t i = 0; i < ch.clouds.size(); i++)
	{
		std::cout << "Cloud " << i <<
			" has " << ch.clouds[i]->points.size() <<
			" points" << std::endl;		
	}

	XYZcloudPtr pc(new XYZcloud);
	XYZcloudPtr bg(new XYZcloud);
	pcl::io::loadPCDFile(PATH + "\\frames\\frame19000.pcd", *pc);
	pcl::io::loadPCDFile(PATH + "\\background.pcd", *bg);

	// Octree resolution - side length of octree voxels
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	XYZcloudPtr diffCloud(new XYZcloud);
	pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
	sdiff.setInputCloud(pc);
	sdiff.setTargetCloud(bg);
	sdiff.setSearchMethod(kdtree);
	sdiff.setDistanceThreshold(tol);
	sdiff.segment(*diffCloud);

	VizPtr viewer;
	//viewer = scroll(pc);
	viewer = scrolldouble(pc, diffCloud);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}