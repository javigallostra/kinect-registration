#ifndef typedefs_h
#define typedefs_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> Normals;

typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

typedef pcl::FPFHSignature33 Feature;
typedef pcl::PointCloud<Feature> Features;

typedef pcl::visualization::PCLVisualizer Visualizer;

#endif