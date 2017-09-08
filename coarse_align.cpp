#include "coarse_align.h"


void CoarsePairwiseAligner::computeNormals (PointCloud::Ptr cloud, Normals::Ptr normals_out)
{
	// Prepare the normal estimator
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setNumberOfThreads(8);
	normal_estimator.setRadiusSearch(0.03);
	// Compute the normals
	normal_estimator.compute(*normals_out);
}

void CoarsePairwiseAligner::computeKeypoints (PointCloud::Ptr cloud, Normals::Ptr cloud_normals,
	PointCloud::Ptr keypoints_out)
{
	kp_detector.computeKeypoints(cloud, keypoints_out, cloud_normals);
}

void CoarsePairwiseAligner::computeFeatures (PointCloud::Ptr cloud, Normals::Ptr normals,
	PointCloud::Ptr keypoints, Features::Ptr features_out)
{
	// Prepare estimation
	feature_estimator.setSearchSurface(cloud);
	feature_estimator.setInputNormals(normals);
	feature_estimator.setRadiusSearch(0.03);
	feature_estimator.setInputCloud(keypoints);
	feature_estimator.setNumberOfThreads(4);
	// Compute features
	feature_estimator.compute(*features_out);
	// Purge features
	purgeFeatures(features_out, keypoints);
}

void CoarsePairwiseAligner::purgeFeatures (Features::Ptr features, PointCloud::Ptr keypoints)
{
	Features::Ptr purgedFeatures (new Features);
	PointCloud::Ptr purgedKeypoints (new PointCloud);
	bool notnan;
	// Loop histogram of each feature
	for (int i = 0; i < features->size(); i++)
	{
		notnan = true;
		// Check for nan values
		for (int j = 0; j < 33; j++)
		{
			if (std::isnan(features->points[i].histogram[j]))
			{
				notnan = false;
				break;
			}
		}
		// If notnan, add to purged
		if (notnan)
		{
			purgedFeatures->push_back(features->points[i]);
			purgedKeypoints->push_back(keypoints->points[i]);
		}
	}
	// Copy purged clouds to original clouds
	*features = *purgedFeatures;
	*purgedKeypoints = *keypoints;
}

void CoarsePairwiseAligner::estimateTransform ()
{
	// Get all correspondences
	correspondence_estimator.setInputSource(source_features);
	correspondence_estimator.setInputTarget(target_features);
	//correspondence_estimator.determineCorrespondences(*correspondences);
	correspondence_estimator.determineReciprocalCorrespondences(*correspondences);
	if (verbose) {std::cout << "Correspondences before rejection: " << correspondences->size() << std::endl;}
	// SAC bad correspondence rejector
	correspondence_rejector.setInputSource(source_keypoints);
	correspondence_rejector.setInputTarget(target_keypoints);
	correspondence_rejector.setInlierThreshold(0.2);
	correspondence_rejector.setMaximumIterations(1000);
	correspondence_rejector.setRefineModel(true);//false
	correspondence_rejector.setInputCorrespondences(correspondences);
	correspondence_rejector.getCorrespondences(*correspondences_filtered);
	if (verbose) { std::cout << "Correspondences after rejection: " << correspondences_filtered->size() << std::endl;}
	// Estimate final transformation
	transformation_estimator.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences_filtered, transformation_matrix);
}

CoarsePairwiseAligner::CoarsePairwiseAligner (bool verb)
{
	verbose = verb;
	kd_tree.reset(new pcl::search::KdTree<PointT>);
	correspondences.reset(new pcl::Correspondences);
	correspondences_filtered.reset(new pcl::Correspondences);
	source_normals.reset(new Normals);
	target_normals.reset(new Normals);
	source_keypoints.reset(new PointCloud);
	target_keypoints.reset(new PointCloud);
	source_features.reset(new Features);
	target_features.reset(new Features);
	transformation_matrix = Eigen::Matrix4f::Identity();
}

void CoarsePairwiseAligner::align (PointCloud::Ptr source_cloud, PointCloud::Ptr target_cloud)
{
	// Reset pointers
	correspondences.reset(new pcl::Correspondences);
	correspondences_filtered.reset(new pcl::Correspondences);
	source_normals.reset(new Normals);
	target_normals.reset(new Normals);
	source_keypoints.reset(new PointCloud);
	target_keypoints.reset(new PointCloud);
	source_features.reset(new Features);
	target_features.reset(new Features);
	// Coarse Align Pipeline
	// 1 - Compute normals
	if (verbose) {std::cout << "Computing normals..." << std::endl;}
	computeNormals(source_cloud, source_normals);
	if (verbose) {std::cout << "Source nm: " << source_normals->size() << std::endl;}
	computeNormals(target_cloud, target_normals);
	if (verbose) {std::cout << "Target nm: " << target_normals->size() << std::endl;}
	// 2 - Compute keypoints
	if (verbose) {std::cout << "Computing keypoints..." << std::endl;}
	computeKeypoints(source_cloud, source_normals, source_keypoints);
	if (verbose) {std::cout << "Source kp: " << source_keypoints->size() << std::endl;}
	computeKeypoints(target_cloud, target_normals, target_keypoints);
	if (verbose) {std::cout << "Target kp: " << target_keypoints->size() << std::endl;}
	// 3 - Compute feature descriptors for the keypoints
	if (verbose) {std::cout << "Computing features..." << std::endl;}
	computeFeatures(source_cloud, source_normals, source_keypoints, source_features);
	if (verbose) {std::cout << "Source ft: " << source_features->size() << std::endl;}
	computeFeatures(target_cloud, target_normals, target_keypoints, target_features);
	if (verbose) {std::cout << "Target ft: " << target_features->size() << std::endl;}
	// 4 - Estimate transformation matrix and return
	if (verbose) {std::cout << "Estimating transform..." << std::endl;}
	estimateTransform();
	if (verbose) {std::cout << "Transformation matrix:" << std::endl;}
	if (verbose) {std::cout << transformation_matrix << std::endl;}
}

void CoarsePairwiseAligner::alignToLast (PointCloud::Ptr source_cloud)
{
	// Last source becomes target
	target_normals.reset(new Normals);
	target_keypoints.reset(new PointCloud);
	target_features.reset(new Features);
	pcl::copyPointCloud(*source_normals, *target_normals);
	pcl::copyPointCloud(*source_keypoints, *target_keypoints);
	pcl::copyPointCloud(*source_features, *target_features);
	// Reset pointers
	source_normals.reset(new Normals);
	source_keypoints.reset(new PointCloud);
	source_features.reset(new Features);
	correspondences.reset(new pcl::Correspondences);
	correspondences_filtered.reset(new pcl::Correspondences);
	// Coarse Align Pipeline
	// 1 - Compute normals
	if (verbose) {std::cout << "Computing source normals..." << std::endl;}
	computeNormals(source_cloud, source_normals);
	if (verbose) {std::cout << "Source nm: " << source_normals->size() << " | Target nm: " << target_normals->size() << std::endl;}
	// 2 - Compute keypoints
	if (verbose) {std::cout << "Computing source keypoints..." << std::endl;}
	computeKeypoints(source_cloud, source_normals, source_keypoints);
	if (verbose) {std::cout << "Source kp: " << source_keypoints->size() << " | Target kp: " << target_keypoints->size() << std::endl;}
	// 3 - Compute feature descriptors for the keypoints
	if (verbose) {std::cout << "Computing source features..." << std::endl;}
	computeFeatures(source_cloud, source_normals, source_keypoints, source_features);
	if (verbose) {std::cout << "Source ft: " << source_features->size() << " | Target ft: " << target_features->size() << std::endl;}
	// 4 - Estimate transformation matrix and return
	if (verbose) {std::cout << "Estimating transform..." << std::endl;}
	estimateTransform();
	if (verbose) {std::cout << "Transformation matrix:" << std::endl;}
	if (verbose) {std::cout << transformation_matrix << std::endl;}
}

Eigen::Matrix4f CoarsePairwiseAligner::getFinalTransformation ()
{
	return transformation_matrix;
}

PointCloud::Ptr CoarsePairwiseAligner::getSourceKeypoints ()
{
	return source_keypoints;
}

PointCloud::Ptr CoarsePairwiseAligner::getTargetKeypoints ()
{
	return target_keypoints;
}

pcl::CorrespondencesPtr CoarsePairwiseAligner::getCorrespondences ()
{
	return correspondences;
}

pcl::CorrespondencesPtr CoarsePairwiseAligner::getFilteredCorrespondences ()
{
	return correspondences_filtered;
}

Features::Ptr CoarsePairwiseAligner::getSourceFeatures ()
{
	return source_features;
}

Features::Ptr CoarsePairwiseAligner::getTargetFeatures ()
{
	return target_features;
}

Normals::Ptr CoarsePairwiseAligner::getSourceNormals ()
{
	return source_normals;
}

Normals::Ptr CoarsePairwiseAligner::getTargetNormals ()
{
	return target_normals;
}

std::string CoarsePairwiseAligner::getKeypointDetectorType ()
{
	return kp_detector.getKeypointType();
}

void CoarsePairwiseAligner::setKeypointDetectorType (std::string type)
{
	kp_detector.setKeypointType(type);
}
