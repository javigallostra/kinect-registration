#include "typedefs.h"

#include <pcl/filters/filter.h>

#include <pcl/point_types.h>

//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
// http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images

#include "kp_detectors.h"

#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

class CoarsePairwiseAligner
{
	private:
		bool verbose;
		Normals::Ptr source_normals, target_normals;
		PointCloud::Ptr source_keypoints, target_keypoints;
		Features::Ptr source_features, target_features;
		pcl::CorrespondencesPtr correspondences, correspondences_filtered;
		pcl::NormalEstimationOMP<PointT, NormalT> normal_estimator;
		pcl::search::KdTree<PointT>::Ptr kd_tree;
		pcl::FPFHEstimationOMP<PointT, NormalT, Feature> feature_estimator;
		KeypointDetector kp_detector;
		pcl::registration::CorrespondenceEstimation<Feature, Feature> correspondence_estimator;
		pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> correspondence_rejector;
		pcl::registration::TransformationEstimationSVD<PointT, PointT> transformation_estimator;
		Eigen::Matrix4f transformation_matrix;

		void computeNormals (PointCloud::Ptr cloud, Normals::Ptr normals_out);

		void computeKeypoints (PointCloud::Ptr cloud, Normals::Ptr cloud_normals,
			PointCloud::Ptr keypoints_out);

		void computeFeatures (PointCloud::Ptr cloud, Normals::Ptr normals,
			PointCloud::Ptr keypoints, Features::Ptr features_out);

		void purgeFeatures (Features::Ptr features, PointCloud::Ptr keypoints);

		void estimateTransform ();

	public:
		CoarsePairwiseAligner (bool verb);

		void align (PointCloud::Ptr source_cloud, PointCloud::Ptr target_cloud);

		void alignToLast (PointCloud::Ptr source_cloud);

		Eigen::Matrix4f getFinalTransformation ();

		PointCloud::Ptr getSourceKeypoints ();

		PointCloud::Ptr getTargetKeypoints ();

		pcl::CorrespondencesPtr getCorrespondences ();

		pcl::CorrespondencesPtr getFilteredCorrespondences ();

		Features::Ptr getSourceFeatures ();

		Features::Ptr getTargetFeatures ();

		Normals::Ptr getSourceNormals ();

		Normals::Ptr getTargetNormals ();

		std::string getKeypointDetectorType ();

		void setKeypointDetectorType (std::string type);
};