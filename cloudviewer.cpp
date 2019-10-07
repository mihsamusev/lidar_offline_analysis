// native c++ includes
#include <string>
#include <iostream>

// project includes
#include <cloudframes.h>
#include <cloudviewer.h>

// 3rd party inlcudes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> // temporary, just for BG

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

// initialize viewer
void CloudViewer::initView()
{
	this->viewer->setBackgroundColor(0, 0, 0);
	this->viewer->addCoordinateSystem(1.0); // scale = 1.0
	this->viewer->initCameraParameters();
	this->viewer->addPointCloud<pcl::PointXYZ>(this->currentCloudPtr, this->CURRENT_CLOUD_LABEL);

	void* cookie = nullptr; // all extra user data is send as void pointer (PCL)
	this->viewer->registerKeyboardCallback(&CloudViewer::keyboardEventOccurred, *this, cookie);
	this->viewer->registerPointPickingCallback(&CloudViewer::pointPickingEventOccurred, *this, cookie);
}

void CloudViewer::initView(std::string& camfile)
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
		this->viewer->updatePointCloud(this->currentCloudPtr,
									   this->CURRENT_CLOUD_LABEL);
		std::cout << "cap: " << this->currentCloudPtr->points.capacity() <<
			" size: " << this->currentCloudPtr->points.size() << std::endl;


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
		this->viewer->updatePointCloud(this->currentCloudPtr,
										this->CURRENT_CLOUD_LABEL);
	}
}

// keys put next and prev frames to viewer for visualization
void CloudViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* ud)
{
	if (event.getKeySym() == "Right" && event.keyDown())
	{
		this->loadNextFrame();
	}
	else if (event.getKeySym() == "Left" && event.keyDown())
	{
		this->loadPrevFrame();
	}
	else if (event.getKeySym() == "m" && event.keyDown())
	{
		
		XYZcloudPtr plane(new XYZcloud);
		this->frames->fitPlane(this->currentCloudPtr, plane, 0.2);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(plane, 255, 0, 255);
		this->viewer->addPointCloud<pcl::PointXYZ>(plane, col, "plane");
		this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");
	}
	else if (event.getKeySym() == "n" && event.keyDown())
	{
		
		XYZcloudPtr bg(new XYZcloud);
		XYZcloudPtr nobg(new XYZcloud);

		pcl::io::loadPCDFile("C:/Users/msa/Documents/cpp/pcl_analysis/data/background.pcd", *bg);
		this->frames->segmentBG(bg, this->currentCloudPtr, nobg, 0.1, 1);

		std::cout << "cap: " << nobg->points.capacity() <<
			" size: " << nobg->points.size() << std::endl;

		//this->viewer->removePointCloud(this->CURRENT_CLOUD_LABEL);
		//this->viewer->addPointCloud(nobg, this->CURRENT_CLOUD_LABEL);
		bool flag = this->viewer->updatePointCloud(nobg, this->CURRENT_CLOUD_LABEL);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(nobg, 0, 255, 0);
		//this->viewer->addPointCloud<pcl::PointXYZ>(nobg, col, "nobg");
		//this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "nobg");
	}
	else if (event.getKeySym() == "b" && event.keyDown())
	{
		this->viewer->updatePointCloud(this->currentCloudPtr, this->CURRENT_CLOUD_LABEL);
	}
}

void CloudViewer::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* ud)
{
	float x, y, z;
	if (event.getPointIndex() == -1)
	{
		return;
	}
	event.getPoint(x, y, z);
	pcl::PointXYZ center;
	center.x = x;
	center.y = y;
	center.z = z;

	this->viewer->removeShape("sphere");
	this->viewer->addSphere(center, 0.1, 0.3, 0.5, 0.3, "sphere");

	std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

