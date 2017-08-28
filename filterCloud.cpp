#include "filterCloud.h"

void cloudFilters::radiusOutlierRemovalFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
    // 1- Build the filter
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.03);
    outrem.setMinNeighborsInRadius(20);
    // 2 - Apply filter
    outrem.filter (*cloud_out);
    if (verbose) {std::cout << "Cloud size after RadiusOutlierRemoval filter: " << cloud_out->size() << std::endl;}
}

void cloudFilters::passthroughFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
    // 1- Build the filter
    ptfilter.setInputCloud (cloud_in);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (0.0, 2.0);
    // 2 - Apply filter
    ptfilter.filter (*cloud_out);
    if (verbose) {std::cout << "Cloud size after PassThrough filter: " << cloud_out->size() << std::endl;}
}

void cloudFilters::voxelFilter(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
    // 1 - Build filter
    voxel_downsampler.setInputCloud (cloud_in);
    voxel_downsampler.setLeafSize (0.01f, 0.01f, 0.01f);
    // 2 - Apply filter
    voxel_downsampler.filter (*cloud_out);
    if (verbose) { std::cout << "Cloud size after VoxelGrid filter: " << cloud_out->size() << std::endl;}
}

void cloudFilters::applyAllFilters(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
    // 1 - Make a copy of the cloud
    PointCloud::Ptr cloud_handler (new PointCloud);
    pcl::copyPointCloud(*cloud_in, *cloud_handler);
    // 2 - Remove NaN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_handler, *cloud_handler, indices);
    if (verbose) {std::cout << "Not NaN points: " << cloud_handler->size() << std::endl;}
    // 3 - Apply first filter
    PointCloud::Ptr first_filtered (new PointCloud);
    passthroughFilter(cloud_handler, first_filtered);
    // 4 - Apply second filter
    PointCloud::Ptr second_filtered (new PointCloud);
    voxelFilter(first_filtered, second_filtered);
    // 5 - Apply third filter
    radiusOutlierRemovalFilter(second_filtered, cloud_out);
}

void cloudFilters::setVerbose (bool verb)
{
    verbose = verb;
}

cloudFilters::cloudFilters ()
{
    verbose = false;
}