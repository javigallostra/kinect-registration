#include "kp_detectors.h"

void KeypointDetector::computeSIFT3DKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out, Normals::Ptr cloud_normals)
{
	SIFT3D_detector.setSearchMethod(kd_tree);
	SIFT3D_detector.setScales(0.03f, 3, 4);
	SIFT3D_detector.setMinimumContrast(0.001f);
	SIFT3D_detector.setInputCloud(cloud);
	// Compute keypoints
	SIFT3D_detector.compute(*keypoints_out);
}

void KeypointDetector::computeISS3DKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out)
{
	// ISS3D parameters
	double model_resolution = computeResolution(cloud);
	double salient_radius = 6 * model_resolution;
	double non_max_radius = 4 * model_resolution;
	double normal_radius = 4 * model_resolution;
	double border_radius = 1 * model_resolution;
	double gamma_21 = 0.975;
	double gamma_32 = 0.975;
	double min_neighbors = 5;
	int threads = 1;
	// Prepare detector
	ISS3D_detector.setSearchMethod (kd_tree);
	ISS3D_detector.setSalientRadius (salient_radius);
	ISS3D_detector.setNonMaxRadius (non_max_radius);
	ISS3D_detector.setNormalRadius (normal_radius);
	ISS3D_detector.setBorderRadius (border_radius);
	ISS3D_detector.setThreshold21 (gamma_21);
	ISS3D_detector.setThreshold32 (gamma_32);
	ISS3D_detector.setMinNeighbors (min_neighbors);
	ISS3D_detector.setNumberOfThreads (threads);
	ISS3D_detector.setInputCloud (cloud);
	// Compute Keypoints
	ISS3D_detector.compute (*keypoints_out);
}

double KeypointDetector::computeResolution (PointCloud::Ptr cloud)
{
	// From http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (! pcl_isfinite (cloud->points[i].x))
		{
	  		continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
	  		res += sqrt (sqr_distances[1]);
	  		++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

KeypointDetector::KeypointDetector ()
{
	// Set SIFT3D by default
	keypoints_type = "ISS3D";
}

void KeypointDetector::computeKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out, Normals::Ptr cloud_normals)
{
	// 1 - Compute Keypoints
	if (keypoints_type == "SIFT3D")
	{
		computeSIFT3DKeypoints(cloud, keypoints_out, cloud_normals);
	}
	else if (keypoints_type == "ISS3D")
	{
		computeISS3DKeypoints(cloud, keypoints_out);
	}
	// 2 - Filter NaN keypoints
	std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*keypoints_out, *keypoints_out, indices);
}

void KeypointDetector::setKeypointType (std::string kp_type)
{
	keypoints_type = kp_type;
}

std::string KeypointDetector::getKeypointType ()
{
	return keypoints_type;
}