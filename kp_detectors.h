#include "typedefs.h"
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/sift_keypoint.h>
// https://github.com/otherlab/pcl/blob/master/examples/keypoints/example_sift_normal_keypoint_estimation.cpp
#include <pcl/keypoints/iss_3d.h>
// http://pointclouds.org/blog/gsoc12/gballin/iss.php
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

class KeypointDetector
{
	private:
		std::string keypoints_type;
		pcl::search::KdTree<PointT>::Ptr kd_tree;
		pcl::SIFTKeypoint<PointT, PointT> SIFT3D_detector;
		pcl::ISSKeypoint3D<PointT, PointT> ISS3D_detector;
		pcl::NarfKeypoint NARF_detector;
		
		void computeSIFT3DKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out, Normals::Ptr cloud_normals);

		void computeISS3DKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out);

		void computeNARFKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out);

		double computeResolution (PointCloud::Ptr cloud);

	public:

		KeypointDetector ();

		void computeKeypoints (PointCloud::Ptr cloud, PointCloud::Ptr keypoints_out, Normals::Ptr cloud_normals);

		void setKeypointType (std::string kp_type);

		std::string getKeypointType ();
};