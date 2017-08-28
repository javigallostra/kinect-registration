#include "typedefs.h"
#include "coarse_align.h"
#include "acquisition.h"
#include "visualization.h"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//------------------------------------//
// Load a .pcd file into a PointCloud //
//------------------------------------//
void load_cloud (std::string filename, PointCloud::Ptr cloud, bool verbose)
{
	pcl::io::loadPCDFile<PointT> (filename, *cloud);

	if (verbose)
	{
		std::cout << "Loaded " << cloud->size() << " data points from " << filename
	        	<< " | width = " << cloud->width << " | height = " << cloud->height
	            << " | isOrganized = " << cloud->isOrganized() << std::endl;
	}
}


//------------------------//
// Handle Keyboard events //
//------------------------//
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* pressedID_void)
{
	if (event.keyDown())
	{
		int* pressedID = static_cast<int*>(pressedID_void);
		std::string key = event.getKeySym();
		if (key == "Return")
		{
			*pressedID = 1;
		}
		else if (key == "Escape")
		{
			*pressedID = 2;
		}
		else
		{
			*pressedID = 0;
		}
	}
}


//--------------------------------//
// Create viewer with 3 viewports //
//--------------------------------//
Visualizer::Ptr create_viewer(int *v1, int *v2, int *v3, int *v4, int *v5, int *v6)
{
	Visualizer::Ptr viewer (new Visualizer ("Cloud Viewer"));
	viewer->initCameraParameters ();
	// Viewport 1 (capture)
	viewer->createViewPort (0.0, 0.5, 0.5, 1.0, *v1);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, *v1);
	viewer->addText ("Capture", 10, 10, "v1text", *v1);
	// Viewport 2 (target cloud)
	viewer->createViewPort (0.5, 0.5, 0.75, 1.0, *v2);
	viewer->setBackgroundColor (0, 0, 0, *v2);
	viewer->addText ("Target", 10, 10, "v2text", *v2);
	// Viewport 3 (source cloud)
	viewer->createViewPort (0.5, 0.0, 0.75, 0.5, *v3);
	viewer->setBackgroundColor (0, 0, 0, *v3);
	viewer->addText ("Source", 10, 10, "v3text", *v3);
	// Viewport 4 (correspondences)
	viewer->createViewPort (0.75, 0.5, 1.0, 1.0, *v4);
	viewer->setBackgroundColor (0, 0, 0, *v4);
	viewer->addText ("Correspondences", 10, 10, "v4text", *v4);
	// Viewport 5 (final correspondences)
	viewer->createViewPort (0.75, 0.0, 1.0, 0.5, *v5);
	viewer->setBackgroundColor (0, 0, 0, *v5);
	viewer->addText ("Final corresp.", 10, 10, "v5text", *v5);
	// Viewport 6 (composition)
	viewer->createViewPort (0.0, 0.0, 0.5, 0.5, *v6);
	viewer->setBackgroundColor (0, 0, 0, *v6);
	viewer->addText ("Composition", 10, 10, "v6text", *v6);
	// Make camera 1 independent from the rest
	viewer->createViewPortCamera (*v1);
	// Position camera
	double zer = 0;
	double neg = -1;
	double pos = 1;
	viewer->setCameraPosition (zer, zer, -4*pos, zer, neg, zer, *v1);
	viewer->setCameraPosition (zer, zer, -4*pos, zer, neg, zer, *v2);
	// Coordinate system
	viewer->addCoordinateSystem (0.25, "v1coord", *v1);
	return viewer;
}


//------//
// Main //
//------//
int main (int argc, char** argv)
{
	PointCloud::Ptr sourceCloud (new PointCloud);
	PointCloud::Ptr targetCloud (new PointCloud);
	PointCloud::Ptr transSourceCloud (new PointCloud);
	PointCloud::Ptr composCloud (new PointCloud);
	PointCloud::Ptr sourceKp (new PointCloud);
	PointCloud::Ptr targetKp (new PointCloud);
	Eigen::Matrix4f transformation;
	int vgrab(0), vtarg(0), vsour(0), vcorr(0), vfcorr(0), vcomp(0), pressedID(0);
	Visualizer::Ptr viewer = create_viewer (&vgrab, &vtarg, &vsour, &vcorr, &vfcorr, &vcomp);
	ImageGrabber grabber (false);
	CoarsePairwiseAligner aligner (false, true);

	// Register callback
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pressedID);

	// Capture first cloud
	PointCloud::Ptr grabberCloud (new PointCloud);
	grabber.captureFrame();
	grabberCloud = grabber.getCurrentFrameCloud();
	// Display first cloud: grabber
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> grabberRGB (grabberCloud);
    viewer->addPointCloud<PointT> (grabberCloud, grabberRGB, "grabberCloud", vgrab);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "grabberCloud");
    // Display first cloud: target
    pcl::copyPointCloud(*grabberCloud, *targetCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> targetRGB (targetCloud);
    viewer->addPointCloud<PointT> (targetCloud, targetRGB, "targetCloud", vtarg);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "targetCloud");
    // Display first cloud: source
    pcl::copyPointCloud(*grabberCloud, *sourceCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sourceRGB (sourceCloud);
    viewer->addPointCloud<PointT> (sourceCloud, sourceRGB, "sourceCloud", vsour);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sourceCloud");
    // Display first cloud: compo
    pcl::copyPointCloud(*grabberCloud, *composCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> composRGB (composCloud);
    viewer->addPointCloud<PointT> (composCloud, composRGB, "composCloud", vcomp);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "composCloud");

    // Main loop
	while (pressedID != 2)
	{
		// When Enter, process cloud
		if (pressedID == 1)
		{
			// 1 - Source is now target
			pcl::copyPointCloud(*sourceCloud, *targetCloud);
			viewer->updatePointCloud(targetCloud, "targetCloud");
			// 2 - New source
			grabber.captureFrame();
			sourceCloud = grabber.getCurrentFrameCloud();
			viewer->updatePointCloud(sourceCloud, "sourceCloud");
			// 3 - Coarse align
			aligner.align(sourceCloud, targetCloud);
			transformation = aligner.getFinalTransformation();
			//////
			sourceKp = aligner.getSourceKeypoints();
			pcl::visualization::PointCloudColorHandlerCustom<PointT> src_kp_color (sourceKp, 0, 255, 0);
        	viewer->addPointCloud<PointT> (sourceKp, src_kp_color, "sourceKp", vsour);
        	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sourceKp");
        	targetKp = aligner.getTargetKeypoints();
			pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_kp_color (targetKp, 0, 255, 0);
        	viewer->addPointCloud<PointT> (targetKp, tgt_kp_color, "targetKp", vtarg);
        	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "targetKp");
			//////
			// 4 - Fine align
			// 5 - Compose final cloud
			pcl::transformPointCloud(*sourceCloud, *transSourceCloud, transformation);
			*composCloud += *transSourceCloud;
			viewer->updatePointCloud(composCloud, "composCloud");
			pressedID = 0;
		}
		grabber.captureFrame();
		grabberCloud = grabber.getCurrentFrameCloud();
		viewer->updatePointCloud(grabberCloud, "grabberCloud");
		viewer->spinOnce(10);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
	std::cout << "Escape pressed, exiting program." << std::endl;
	return(0);
}