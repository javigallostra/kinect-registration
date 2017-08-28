#include "typedefs.h"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class FineAligner
{
	private:
		pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> icp;
    	PointCloudNormal::Ptr transformed_cloud;
    	Eigen::Matrix4f transformation_matrix;
    	bool verbose;

    public:

    	void align (PointCloudNormal::Ptr source_cloud, PointCloudNormal::Ptr target_cloud);

    	Eigen::Matrix4f getFinalTransformation ();

    	FineAligner (bool verb);
};