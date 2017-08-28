#include "fine_align.h"

void FineAligner::align (PointCloudNormal::Ptr source_cloud, PointCloudNormal::Ptr target_cloud)
{
    // Align
    if (verbose) {std::cout << "Fine align with ICP..." << std::endl;}
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.align(*transformed_cloud);
    transformation_matrix = icp.getFinalTransformation();
    // Print
    if (verbose) {std::cout << "Transformation matrix:" << std::endl;}
    if (verbose) {std::cout << transformation_matrix << std::endl;}
}

Eigen::Matrix4f FineAligner::getFinalTransformation ()
{
    return transformation_matrix;
}

FineAligner::FineAligner (bool verb)
{
    // Initialize variables
    verbose = verb;
    transformation_matrix = Eigen::Matrix4f::Identity();
    transformed_cloud.reset(new PointCloudNormal);
    // Set ICP criteria
    icp.setMaxCorrespondenceDistance (200);
    icp.setMaximumIterations (100);
    icp.setTransformationEpsilon (1e-9);
    icp.setEuclideanFitnessEpsilon (10);
}
