#include "fine_align.h"

void FineAligner::align (PointCloudNormal::Ptr source_cloud, PointCloudNormal::Ptr target_cloud, Eigen::Matrix4f transformation_guess)
{
    // Align
    if (verbose) {std::cout << "Fine align with ICP..." << std::endl;}
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.align(*transformed_cloud, transformation_guess);
    transformation_matrix = icp.getFinalTransformation();
    // Print
    if (verbose) {std::cout << "Transformation matrix:" << std::endl;}
    if (verbose) {std::cout << transformation_matrix << std::endl;}
}

Eigen::Matrix4f FineAligner::getFinalTransformation ()
{
    return transformation_matrix;
}

void FineAligner::setMaximumIterations (int max_iterations)
{
    icp.setMaximumIterations(max_iterations);
}

FineAligner::FineAligner (bool verb)
{
    // Initialize variables
    verbose = verb;
    transformation_matrix = Eigen::Matrix4f::Identity();
    transformed_cloud.reset(new PointCloudNormal);
    // Set ICP criteria
    icp.setMaxCorrespondenceDistance (0.1);
    icp.setRANSACOutlierRejectionThreshold(0.001);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon (1e-8);
}
