#include "surface_reconstruction.h"

surfaceReconstructor::surfaceReconstructor (bool verb)
{
	verbose = verb;
	method = "GREEDY";
	kd_tree.reset(new pcl::search::KdTree<PointNormal>);
	// MLS parameters
	mls.setComputeNormals(true);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setSearchRadius(0.03);
	// Poisson parameters
	poisson.setDepth(10);
	poisson.setSamplesPerNode(10);
	// Greedy parameters
	gp3.setSearchRadius(0.1); // maximum triangle edge length
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (1000);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);
}

void surfaceReconstructor::reconstruct (PointCloud::Ptr cloud_in, PointCloudNormal::Ptr cloud_out, pcl::PolygonMesh::Ptr mesh_out)
{
	PointCloudNormal::Ptr cloud_handler (new PointCloudNormal);
	// 1 - Apply MLS to smooth surface
	if (verbose) {std::cout << "Applying MLS to cloud..." << std::endl;}
	mls.setInputCloud(cloud_in);
	mls.process(*cloud_handler);
	// 2 - Voxelize cloud
    voxel_downsampler.setInputCloud (cloud_handler);
    voxel_downsampler.setLeafSize (0.01f, 0.01f, 0.01f);
    voxel_downsampler.filter (*cloud_out);
	// 2 - Apply algorithm to generate mesh
	if (verbose) {std::cout << "Generating mesh..." << std::endl;}
	if (method == "POISSON")
	{
	    poisson.setInputCloud(cloud_out);
		poisson.reconstruct(*mesh_out);
	}
	else if (method == "GREEDY")
	{
		kd_tree->setInputCloud(cloud_out);
		gp3.setInputCloud(cloud_out);
		gp3.setSearchMethod(kd_tree);
		gp3.reconstruct(*mesh_out);
	}
	if (verbose) {std::cout << "Mesh finished." << std::endl;}
}