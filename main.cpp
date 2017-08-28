#include "typedefs.h"
#include "coarse_align.h"
#include "acquisition.h"
#include "visualization.h"
#include "filterCloud.h"
#include "fine_align.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <dirent.h>
#include <ctime>
#include <sstream>

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

//---------------------------------//
// Load filenames from a directory //
//---------------------------------//

void get_files_from_dir (std::string directory_path, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    std::string filename;
    if((dp  = opendir(directory_path.c_str())) != NULL)
    {
    	while ((dirp = readdir(dp)) != NULL)
    	{
    		filename = std::string(dirp->d_name);
    		if (filename != ".." and filename != ".")
    		{
	        	files.push_back(filename);
	    	}
	    }
	    closedir(dp);
    }
}


//------//
// Main //
//------//
int main (int argc, char** argv)
{
	// Clock
	std::clock_t start;
	double seconds;
	std::ostringstream fps_str;
	std::string fps;
	// Default settings
	bool capture = true;
	bool load_files = false;
	bool save_files = false;
	bool verbose = false;
	std::string kp_type = "ISS3D";
	// Parse command-line arguments
	if (argc > 1)
	{
		for (int i = 1; i < argc; i++)
		{
			std::string argument = std::string(argv[i]);
			if (argument == "-l")
			{
				load_files = true;
				capture = false;
			}
			else if (argument == "-v")
			{
				verbose = true;
			}
			else if (argument == "-s")
			{
				save_files = true;
			}
			else if (argument == "-k")
			{
				std::string type = std::string(argv[i+1]);
				if (type == "ISS3D" or type == "SIFT3D")
				{
					kp_type = type;
				}
				else
				{
					std::cout << "Unknown keypoint type, using default." << std::endl;
				}
			}
		}
	}
	// Initialize variables
	std::vector<std::string> files_to_load;
	pcl::PCDWriter pcd_writer;
	PointCloud::Ptr grabberCloud (new PointCloud);
	PointCloud::Ptr sourceCloud (new PointCloud);
	PointCloud::Ptr targetCloud (new PointCloud);
	PointCloud::Ptr transSourceCloud (new PointCloud);
	PointCloud::Ptr composCloud (new PointCloud);
	PointCloud::Ptr sourceKp (new PointCloud);
	PointCloud::Ptr targetKp (new PointCloud);
	PointCloudNormal::Ptr sourceNormalCloud (new PointCloudNormal);
	PointCloudNormal::Ptr targetNormalCloud (new PointCloudNormal);
	PointCloudNormal::Ptr transICPNormalCloud (new PointCloudNormal);
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	std::vector<Eigen::Matrix4f> transformations_vector;
	int pictures = 0;
	int last_loaded = 0;
	bool load_more = true;

	Viewer viewerObj (grabberCloud, sourceCloud, targetCloud, composCloud, sourceKp, targetKp);
	Visualizer::Ptr viewer = viewerObj.getViewer();
	CoarsePairwiseAligner coarse_aligner (verbose);
	coarse_aligner.setKeypointDetectorType(kp_type);
	FineAligner fine_aligner (verbose);
	ImageGrabber grabber (false);

	// Load first cloud
	if (capture)
	{
		grabber.captureFrame();
		grabberCloud = grabber.getCurrentFrameCloud();
	}
	else if (load_files)
	{
		get_files_from_dir("./images", files_to_load);
		std::sort(files_to_load.begin(), files_to_load.end());
		load_cloud("./images/" + files_to_load[pictures], grabberCloud, verbose);
	}

    // Main loop
	while (viewerObj.getPressedID() != 2)
	{
		start = std::clock();
		// 1 - When Enter, process cloud
		if (viewerObj.getPressedID() == 1 and load_more)
		{
			if (pictures == 0)
			{
				// Grab first cloud
				pcl::copyPointCloud(*grabberCloud, *targetCloud);
				// Add to composition
				*composCloud += *targetCloud;
			}
			else
			{
				// COARSE ALIGN
				if (pictures == 1)
				{
					// Get source
					pcl::copyPointCloud(*grabberCloud, *sourceCloud);
					// Align
					coarse_aligner.align(sourceCloud, targetCloud);
				}
				else if (pictures > 1)
				{
					// Source is now target
					targetCloud.reset(new PointCloud);
					pcl::copyPointCloud(*sourceCloud, *targetCloud);
					// New source filtered
					sourceCloud.reset(new PointCloud);
					pcl::copyPointCloud(*grabberCloud, *sourceCloud);
					// Align
					coarse_aligner.alignToLast(sourceCloud);
				}
				// FINE ALIGN
				pcl::concatenateFields(*targetCloud, *(coarse_aligner.getTargetNormals()), *targetNormalCloud);
				pcl::concatenateFields(*sourceCloud, *(coarse_aligner.getSourceNormals()), *sourceNormalCloud);
				pcl::transformPointCloud(*sourceNormalCloud, *transICPNormalCloud, coarse_aligner.getFinalTransformation());
				fine_aligner.align(transICPNormalCloud, targetNormalCloud);
				// Get fine transformation
				transformations_vector.push_back(fine_aligner.getFinalTransformation());
				// Get coarse transformation
				transformations_vector.push_back(coarse_aligner.getFinalTransformation());
				// Get keypoint clouds
				sourceKp = coarse_aligner.getSourceKeypoints();
	        	targetKp = coarse_aligner.getTargetKeypoints();
			}
			// Apply previous transformations and compose cloud
			*transSourceCloud = *sourceCloud;
			for (int i = 0; i < transformations_vector.size(); i++)
			{
				transformation = transformations_vector[i];
				pcl::transformPointCloud(*transSourceCloud, *transSourceCloud, transformation);
			}
			*composCloud += *transSourceCloud;
			// Display changes
			viewerObj.updateClouds(sourceCloud, targetCloud, composCloud, sourceKp, targetKp);
			viewerObj.updateCorrespondences(coarse_aligner.getFilteredCorrespondences(), sourceKp, targetKp);
			viewerObj.setPressedID(0);
			pictures += 1;
		}
		// 2 - Load new image
		if (capture)
		{
			grabber.captureFrame();
			grabberCloud = grabber.getCurrentFrameCloud();
		}
		else if (load_files)
		{
			if (pictures >= files_to_load.size())
			{
				grabberCloud.reset(new PointCloud);
				load_more = false;
			}
			else if (pictures != last_loaded)
			{
				load_cloud("./images/" + files_to_load[pictures], grabberCloud, verbose);
				last_loaded = pictures;
			}
		}
		// 3 - Update image and fps
		seconds = (std::clock()- start) / (double) CLOCKS_PER_SEC;
		fps_str.str("");
		fps_str << (1/seconds);
		viewerObj.updateFPS(fps_str.str());
		viewerObj.updateGrabber(grabberCloud);
		viewer->spinOnce(10);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
	std::cout << "Escape pressed, exiting program." << std::endl;
	return(0);
}

// Filters: when to filter...
// Save files for checking ICP!!!!
// Timer