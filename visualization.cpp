#include "visualization.h"

void Viewer::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* pressedID_void)
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
		else if (key == "BackSpace")
		{
			*pressedID = 3;
		}
		else
		{
			*pressedID = 0;
		}
	}
}

void Viewer::displayClouds (PointCloud::Ptr grabberCloud, PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp)
{
	// Display first cloud: grabber
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> grabberRGB (grabberCloud);
    viewer->addPointCloud<PointT> (grabberCloud, grabberRGB, "grabberCloud", vgrab);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "grabberCloud");
    // Display target
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> targetRGB (targetCloud);
    viewer->addPointCloud<PointT> (targetCloud, targetRGB, "targetCloud", vtarg);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "targetCloud");
    // Display source
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sourceRGB (sourceCloud);
    viewer->addPointCloud<PointT> (sourceCloud, sourceRGB, "sourceCloud", vsour);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sourceCloud");
    // Display compo
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> composRGB (composCloud);
    viewer->addPointCloud<PointT> (composCloud, composRGB, "composCloud", vcomp);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "composCloud");
    // Display source kp
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_kp_color (sourceKp, 255, 0, 0);
    viewer->addPointCloud<PointT> (sourceKp, src_kp_color, "sourceKp", vsour);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sourceKp");
    // Display target kp
	pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_kp_color (targetKp, 0, 255, 0);
   	viewer->addPointCloud<PointT> (targetKp, tgt_kp_color, "targetKp", vtarg);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targetKp");
	// Display correspondences
	viewer->addPointCloud<PointT> (targetCloud, targetRGB, "corrtargetCloud", vcorr);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "corrtargetCloud");
	PointCloud::Ptr transformed (new PointCloud);
	pcl::transformPointCloud(*sourceCloud, *transformed, correspondence_cloud_transform);
	viewer->addPointCloud<PointT> (transformed, sourceRGB, "corrsourceCloud", vcorr);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "corrsourceCloud");
}

Viewer::Viewer (PointCloud::Ptr grabberCloud, PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp)
{
	// 1 - Initialize viewer and variables
	viewer.reset(new Visualizer ("Cloud Viewer"));
	viewer->initCameraParameters();
	vgrab, vtarg, vsour, vcorr, vfcorr, vcomp, pressedID = 0;
	correspondence_cloud_transform = Eigen::Matrix4f::Identity();
	correspondence_cloud_transform(0, 3) = 2; // x translation
	// 2 - Create viewports and position cameras
	// Viewport 1 (capture)
	viewer->createViewPort (0.0, 0.5, 0.5, 1.0, vgrab);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, vgrab);
	viewer->addText ("Capture | FPS:", 10, 10, "vgrabtext", vgrab);
	// Viewport 2 (target cloud)
	viewer->createViewPort (0.5, 0.0, 0.75, 0.5, vtarg);
	viewer->setBackgroundColor (0.1, 0.1, 0.1, vtarg);
	viewer->addText ("Target", 10, 10, "vtargtext", vtarg);
	// Viewport 3 (source cloud)
	viewer->createViewPort (0.75, 0.0, 1.0, 0.5, vsour);
	viewer->setBackgroundColor (0.1, 0.1, 0.1, vsour);
	viewer->addText ("Source", 10, 10, "vsourtext", vsour);
	// Viewport 4 (correspondences)
	viewer->createViewPort (0.5, 0.5, 1.0, 1.0, vcorr);
	viewer->setBackgroundColor (0.1, 0.1, 0.1, vcorr);
	viewer->addText ("Correspondences", 10, 10, "vcorrtext", vcorr);
	// Viewport 5 (final correspondences)
	//viewer->createViewPort (0.75, 0.0, 1.0, 0.5, vfcorr);
	//viewer->setBackgroundColor (0, 0, 0, vfcorr);
	//viewer->addText ("Final corresp.", 10, 10, "vfcorrtext", vfcorr);
	// Viewport 6 (composition)
	viewer->createViewPort (0.0, 0.0, 0.5, 0.5, vcomp);
	viewer->setBackgroundColor (0.1, 0.1, 0.1, vcomp);
	viewer->addText ("Composition", 10, 10, "vcomptext", vcomp);
	// Make grabber independent from the rest
	viewer->createViewPortCamera (vgrab);
	// Position cameras
	viewer->setCameraPosition (0, 0, -4, 0, -1, 0, vgrab);
	viewer->setCameraPosition (0, 0, -4, 0, -1, 0, vtarg);
	// Add coordinate system
	viewer->addCoordinateSystem (0.25, "vgrabcoord", vgrab);
	// 3 - Register Keyboard callback
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pressedID);
	// 4 - Display first clouds, even if they are empty
	displayClouds(grabberCloud, sourceCloud, targetCloud, composCloud, sourceKp, targetKp);
}

Visualizer::Ptr Viewer::getViewer ()
{
	return viewer;
}

void Viewer::updateGrabber (PointCloud::Ptr grabberCloud)
{
	viewer->updatePointCloud (grabberCloud, "grabberCloud");
}

void Viewer::updateClouds (PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp)
{
    viewer->updatePointCloud (targetCloud, "targetCloud");
    viewer->updatePointCloud (sourceCloud, "sourceCloud");
    viewer->updatePointCloud (composCloud, "composCloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_kp_color (sourceKp, 255, 0, 0);
    viewer->updatePointCloud (sourceKp, src_kp_color, "sourceKp");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_kp_color (targetKp, 0, 255, 0);
    viewer->updatePointCloud (targetKp, tgt_kp_color, "targetKp");
    viewer->updatePointCloud (targetCloud, "corrtargetCloud");
    PointCloud::Ptr transformed (new PointCloud);
	pcl::transformPointCloud(*sourceCloud, *transformed, correspondence_cloud_transform);
    viewer->updatePointCloud (transformed, "corrsourceCloud");
}

void Viewer::updateCorrespondences (pcl::CorrespondencesPtr correspondences, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp)
{
	// Remove previous
	viewer->removeAllShapes(vcorr);
	// Translation of target keypoints
	PointCloud::Ptr transformed_sourceKp (new PointCloud);
	pcl::transformPointCloud(*sourceKp, *transformed_sourceKp, correspondence_cloud_transform);
	// Correspondence loop drawing
	for (size_t i = 0; i < (*correspondences).size (); ++i)
	{
	    // Get the pair of points
	    const PointT & p_left = transformed_sourceKp->points[(*correspondences)[i].index_query];
	    const PointT & p_right = targetKp->points[(*correspondences)[i].index_match];

	    // Generate a random (bright) color
	    double r = (rand() % 100);
	    double g = (rand() % 100);
	    double b = (rand() % 100);
	    double max_channel = std::max (r, std::max (g, b));
	    r /= max_channel;
	    g /= max_channel;
	    b /= max_channel;

	    // Generate a unique string for each line
	    std::stringstream ss ("line");
	    ss << i;

	    // Draw the line
	    viewer->addLine (p_left, p_right, r, g, b, ss.str(), vcorr);
	}
}

void Viewer::addFinalMesh (pcl::PolygonMesh::Ptr mesh_in)
{
	// 1 - Remove composCloud
	viewer->removeAllPointClouds(vcomp);
	// 2 - Add mesh to compos Viewport
	viewer->addPolygonMesh(*mesh_in, "mesh", vcomp);
}

void Viewer::updateCorrespondenceTransform (Eigen::Matrix4f new_transform)
{
	correspondence_cloud_transform = new_transform;
}

void Viewer::updateFPS (std::string fps)
{
	viewer->updateText("Capture | FPS: " + fps, 10, 10, "vgrabtext");
}

int Viewer::getPressedID ()
{
	return pressedID;
}

void Viewer::setPressedID (int newID)
{
	pressedID = newID;
}