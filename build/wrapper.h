#pragma once
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

extern "C" {
	// Types
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointCloudT;
	typedef pcl::FPFHSignature33 FeatureT;
	typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
	typedef pcl::PointCloud<FeatureT> FeatureCloudT;
	typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
	__declspec(dllexport) int detectplane(float* source, int size, float* transInfo);
	__declspec(dllexport) float* dataConverter(float* source, int size, float* initial_guess, bool isFirst);
	__declspec(dllexport) int initialGuess(PointCloudT::Ptr object, PointCloudT::Ptr scene, Eigen::Matrix4f &transformation);
}
