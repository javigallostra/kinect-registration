#include "typedefs.h"
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

class SurfaceReconstructor
{
	private:
		pcl::MovingLeastSquares<PointT, PointNormal> mls;

		pcl::Poisson<PointNormal> poisson;

		pcl::GreedyProjectionTriangulation<PointNormal> gp3;

		pcl::search::KdTree<PointNormal>::Ptr kd_tree;

		pcl::VoxelGrid<PointNormal> voxel_downsampler;

		std::string method;

		bool verbose;

	public:

		SurfaceReconstructor (bool verb);

		void reconstruct (PointCloud::Ptr cloud_in, PointCloudNormal::Ptr cloud_out, pcl::PolygonMesh::Ptr mesh_out);

		void setReconstructionMethod (std::string new_method);
};