#ifndef FILTERS_H
#define FILTERS_H

#include "typedefs.h"
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class cloudFilters
{
	private:
		bool verbose;

		pcl::RadiusOutlierRemoval<PointT> outrem;

		pcl::PassThrough<PointT> ptfilter;

		pcl::VoxelGrid<PointT> voxel_downsampler;

	public:
		void radiusOutlierRemovalFilter (PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);

		void passthroughFilter (PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);

		void voxelFilter (PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);

		void applyAllFilters (PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);

		void setVerbose (bool verb);

		cloudFilters ();
};

#endif
