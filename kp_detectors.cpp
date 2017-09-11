#include "kp_detectors.h"


void KeypointDetector::computeSIFT3DKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out)
{
	kd_tree->setInputCloud(cloud);
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
	int threads = 4;
	// Prepare detector
	kd_tree->setInputCloud(cloud);
	ISS3D_detector.setSearchMethod(kd_tree);
	ISS3D_detector.setSalientRadius(salient_radius);
	ISS3D_detector.setNonMaxRadius(non_max_radius);
	ISS3D_detector.setNormalRadius(normal_radius);
	ISS3D_detector.setBorderRadius(border_radius);
	ISS3D_detector.setThreshold21(gamma_21);
	ISS3D_detector.setThreshold32(gamma_32);
	ISS3D_detector.setMinNeighbors(min_neighbors);
	ISS3D_detector.setNumberOfThreads(threads);
	ISS3D_detector.setInputCloud(cloud);
	// Compute Keypoints
	ISS3D_detector.compute (*keypoints_out);
}

void KeypointDetector::computeNARFKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out)
{
	// 1 - Create range image
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2]))*Eigen::Affine3f(cloud->sensor_orientation_);
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	float angularResolution = pcl::deg2rad(1/(((512.0f / 70.6f) + (424.0f / 60.0f)) / 2.0f));  // average pixels/radian
	float maxAngleWidth = pcl::deg2rad(70.6f);
	float maxAngleHeight = pcl::deg2rad(60.0f);
	float noiseLevel = 0.0f;
	float minRange = 0.0f;
	int borderSize = 1;
	range_image.createFromPointCloud (*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);
	// 2 - Compute keypoints
	pcl::RangeImageBorderExtractor rangeImageBorderExtractor;
	NARF_detector.setRangeImageBorderExtractor(&rangeImageBorderExtractor);
	NARF_detector.setRangeImage(&range_image);
	NARF_detector.getParameters().support_size = 0.2f;
	NARF_detector.setRadiusSearch(0.03);
	pcl::PointCloud<int> kp_indices;
	NARF_detector.compute(kp_indices);
	// 3 - Fill keypoints_out cloud
	keypoints_out->points.resize(kp_indices.points.size());
	pcl::PointWithRange current_point;
	for (size_t i=0; i<kp_indices.points.size(); ++i)
	{
		keypoints_out->points[i].getVector3fMap() = range_image.points[kp_indices.points[i]].getVector3fMap();
	}
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
		// Considering the second neighbor since the first is the point itself.
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
	kd_tree.reset(new pcl::search::KdTree<PointT>);
}

void KeypointDetector::computeKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out)
{
	// 1 - Compute Keypoints
	if (keypoints_type == "SIFT3D")
	{
		computeSIFT3DKeypoints(cloud, keypoints_out);
	}
	else if (keypoints_type == "ISS3D")
	{
		computeISS3DKeypoints(cloud, keypoints_out);
	}
	else if (keypoints_type == "NARF")
	{
		computeNARFKeypoints(cloud, keypoints_out);
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