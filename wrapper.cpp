#include <time.h>
#include <string>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "build/wrapper.h"
#include <fstream>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/keyboard_event.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <boost/format.hpp>

typedef pcl::PointXYZ RefPointType;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


using namespace std;

extern "C" {
	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 10.0);
		pass.setKeepOrganized(false);
		pass.setInputCloud(cloud);
		pass.filter(result);
	}

	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
	{
		pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
		grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
		grid.setInputCloud(cloud);
		grid.filter(result);
	}

	int initialGuess(PointCloudT::Ptr object, PointCloudT::Ptr scene, Eigen::Matrix4f &transformation)
	{
		// Point clouds
		//PointCloudT::Ptr object(new PointCloudT);
		PointCloudT::Ptr object_aligned(new PointCloudT);
		//PointCloudT::Ptr scene(new PointCloudT);
		FeatureCloudT::Ptr object_features(new FeatureCloudT);
		FeatureCloudT::Ptr scene_features(new FeatureCloudT);

		//pcl::io::loadPCDFile<PointNT>("chef.pcd", *object);
		//pcl::io::loadPCDFile<PointNT>("rs1.pcd", *scene);
		//pcl::io::savePLYFileASCII<PointNT>("chef.ply", *object);
		//pcl::io::savePLYFileASCII<PointNT>("rs1.ply", *scene);

		std::cout << "object size: " << object->size() << std::endl;
		std::cout << "scene size : " << scene->size() << std::endl;

		// Downsample
		pcl::console::print_highlight("Downsampling...\n");
		pcl::VoxelGrid<PointNT> grid;
		const float leaf = 0.005f;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(object);
		grid.filter(*object);
		grid.setInputCloud(scene);
		grid.filter(*scene);

		std::cout << "object size: " << object->size() << std::endl;
		std::cout << "scene size : " << scene->size() << std::endl;

		// Estimate normals for scene
		pcl::console::print_highlight("Estimating scene normals...\n");
		pcl::NormalEstimationOMP<PointNT, PointNT> nest;
		nest.setRadiusSearch(0.01);
		nest.setInputCloud(scene);
		nest.compute(*scene); 
		nest.setInputCloud(object);
		nest.compute(*object);


		// Estimate features
		pcl::console::print_highlight("Estimating features...\n");
		FeatureEstimationT fest;
		fest.setRadiusSearch(0.025);
		fest.setInputCloud(object);
		fest.setInputNormals(object);
		fest.compute(*object_features);
		fest.setInputCloud(scene);
		fest.setInputNormals(scene);
		fest.compute(*scene_features);

		// Perform alignment
		pcl::console::print_highlight("Starting alignment...\n");
		pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
		align.setInputSource(object);
		align.setSourceFeatures(object_features);
		align.setInputTarget(scene);
		align.setTargetFeatures(scene_features);
		align.setMaximumIterations(50000); // Number of RANSAC iterations
		align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness(5); // Number of nearest features to use
		align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
		align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
		{
			pcl::ScopeTime t("Alignment");
			align.align(*object_aligned);
		}

		if (align.hasConverged())
		{
			// Print results
			printf("\n");
			transformation = align.getFinalTransformation();

		}
		else
		{
			pcl::console::print_error("Alignment failed!\n");
			return (1);
		}

		return (0);
	}

	int detectplane(float* source, int size, float* transInfo)
	{
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
		
	//=============================================initialize================================================
		pcl::PCLPointCloud2 cloud;
		cloud.data.clear();
		cloud.data.resize(size * sizeof(float));

		//uint8_t *start = reinterpret_cast<uint8_t*> (source);

	//===========================================read header file============================================

		pcl::PCDReader reader;
		if (reader.readHeader("bunny.pcd", cloud) == -1) {
			//cout << "read header error" << endl;
			//fout << "read header error\n";
			return (-1);
		}

		memcpy(&cloud.data[0], source, size * sizeof(float));
		cloud.width = (uint32_t)(size / 3);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(cloud, *pc);

	//===========================================segmentation=============================================

		//fout << "after pc2" << pc->points.size() << endl;
		// ransac plane detection
		//创建一个模型参数对象，用于记录结果
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		//inliers表示误差能容忍的点 记录的是点云的序号
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 创建一个分割器
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setMaxIterations(100);
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory-设置目标几何形状
		seg.setModelType(pcl::SACMODEL_PLANE);
		//分割方法：随机采样法
		seg.setMethodType(pcl::SAC_RANSAC);
		//设置误差容忍范围
		seg.setDistanceThreshold(0.0250);
		//输入点云
		seg.setInputCloud(pc);
		//分割点云
		seg.segment(*inliers, *coefficients);
	//==========================================extract plane()================================================
		pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(pc);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*segmentCloud);
	//======================================filter out large z (not used)========================================
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 0.4);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.4, 0.4);
		pass.setKeepOrganized(false);
		pass.setInputCloud(segmentCloud);
		pass.filter(*segmentCloud);
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		//viewer->updatePointCloud(segmentCloud,"cloud");
		//viewer->addCoordinateSystem(0.2);
		////======= Visualization of plane ======
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::copyPointCloud(*pc, *pc_color);
		//uint8_t pc_r = 255, pc_g = 0, pc_b = 0;
		//uint32_t pc_red = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		//pc_r = 0, pc_g = 255, pc_b = 0;
		//uint32_t pc_green = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		/*for (size_t i = 0; i < pc_color->size(); i++)
		{
			pc_color->points[i].rgb = *reinterpret_cast<float*>(&pc_green);
		}
		*/

	//=============================find center of the detected plane and transform================================

		float sum_x=0, sum_y=0, sum_z=0;

		for (size_t i = 0; i < inliers->indices.size(); i++)
		{
			sum_x += pc->points[inliers->indices[i]].x;
			sum_y += pc->points[inliers->indices[i]].y;
			sum_z += pc->points[inliers->indices[i]].z;
		}
		float avg_x = sum_x / inliers->indices.size();
		float avg_y = sum_y / inliers->indices.size();
		float avg_z = sum_z / inliers->indices.size();
		Eigen::Vector3d u(0, 1, 0);
		Eigen::Vector3d v(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2]);
		Eigen::Vector3d temp = u.cross(v);
		double quat_w = u.norm()*v.norm() + u.dot(v);
		Eigen::Vector4d quat;
		quat << temp, quat_w;
		quat.normalize();
		

		transInfo[0] = avg_x;
		transInfo[1] = avg_y;
		transInfo[2] = avg_z;
		transInfo[3] = quat[0];
		transInfo[4] = quat[1];
		transInfo[5] = quat[2];
		transInfo[6] = quat[3];

		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		//viewer->addPointCloud(pc,"cloud");
		//viewer->addCoordinateSystem(0.2);
		Eigen::Matrix4f pose;

		Eigen::Matrix3f rotm;
		rotm = Eigen::Quaternionf((float)quat[3], (float)quat[0], (float)quat[1], (float)quat[2]).toRotationMatrix();
		pose.block<3, 3>(0, 0) = rotm;
		pose.block<3, 1>(0, 3) = Eigen::Vector3f(avg_x, avg_y, avg_z);
		pose.bottomRows(1).setZero();
		
		pose(3, 3) = 1;
		Eigen::Affine3f pose_aff;
		pose_aff.matrix() = pose;

		/*
		viewer->addCoordinateSystem(0.2,pose_aff);
		pcl::PointXYZ p1(avg_x,avg_y,avg_z);
		pcl::PointXYZ p2(avg_x+ coefficients->values[0], avg_y+ coefficients->values[1], avg_z+ coefficients->values[2]);
		//viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(p1,p2,1,0,0,"arrow",0);
		viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(p1, p2);
		//viewer->spinOnce(100);
		*/
		return 0;
	}


	float* dataConverter(float* source, int size, float* initial_guess/*, float* output_pose*/, bool isFirst)
	{
	//=================================================the viewer=============================================================
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		
	//=================================================copy data from Meta to PCL format=======================================
		pcl::PCLPointCloud2 cloud;
		cloud.data.clear();
		cloud.data.resize(size * sizeof(float));
		
		pcl::PCDReader reader;
		if (reader.readHeader("bunny.pcd", cloud) == -1) {return nullptr;}

		memcpy(&cloud.data[0], source, size * sizeof(float));
		cloud.width = (uint32_t)(size / 3);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(cloud, *pc);
	//==================================================plane detection==========================================================
		// create a plane info container
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// set parameter
		seg.setMaxIterations(100);
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.015); //0.025
		seg.setInputCloud(pc);
		// compute
		seg.segment(*inliers, *coefficients);
	//==================================================color plane (optional)================================================
		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*pc, *pc_color);
		
		uint8_t pc_r = 255, pc_g = 0, pc_b = 0;
		uint32_t pc_red = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		pc_r = 0, pc_g = 255, pc_b = 0;
		uint32_t pc_green = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		pc_r = 0, pc_g = 50, pc_b = 255;
		uint32_t pc_blue = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		pc_r = 255, pc_g = 255, pc_b = 80;
		uint32_t pc_yellow = ((uint32_t)pc_r << 16 | (uint32_t)pc_g << 8 | (uint32_t)pc_b);
		
		for (size_t i = 0; i < pc_color->size(); i++)
		{
			pc_color->points[i].rgb = *reinterpret_cast<float*>(&pc_blue);
		}

		for (size_t i = 0; i < inliers->indices.size(); i++)
		{
			pc_color->points[inliers->indices[i]].rgb = *reinterpret_cast<float*>(&pc_red);
		}*/
		//viewer->addPointCloud(pc_color, "pc_rgb");
	//==================================================find the segmentaion==========================================================
		pcl::PointCloud<pcl::PointXYZ>::Ptr above_plane(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr above_plane_ind(new pcl::PointIndices);
		above_plane_ind->indices.clear();
		above_plane->clear();

		float norm_fac = sqrt(coefficients->values[0] * coefficients->values[0] + coefficients->values[1] * coefficients->values[1] + coefficients->values[2] * coefficients->values[2]);
		for (int i = 0; i < pc->points.size();i++)
		{
			auto point = pc->points[i];
			float classifier = point.x*coefficients->values[0] + point.y*coefficients->values[1] + point.z*coefficients->values[2] + coefficients->values[3];
			// flip the plane if facing downward
			if (coefficients->values[2]>0)
			{
				classifier = -classifier;
			}
			//larger z than 0.8 meter not allowed
			if (classifier/norm_fac > 0.018 && point.z < 0.8f  && abs(point.x) < 0.25f && abs(point.y) < 0.20f)
			{
				above_plane_ind->indices.push_back(i);
				above_plane->push_back(point);
			}
		}
		// prevent icp from crashing
		if (above_plane->size()<5)
		{
			float* output_pose = new float[16];
			for (size_t i = 0; i < 15; i++)
			{
				output_pose[i] = -1;
			}
			return output_pose;
		}

		// -------add color to above plane in pc (optional)---------
		/*for (size_t i = 0; i < above_plane_ind->indices.size(); i++)
		{
			pc_color->points[above_plane_ind->indices[i]].rgb = *reinterpret_cast<float*>(&pc_yellow);
		}*/
		//viewer->addPointCloud(pc_color, "pc_rgb");
	//==============================================import cad model ==================================================================

		pcl::PointCloud<pcl::PointXYZ>::Ptr cad_bunny(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPLYFile("bun_zipper_res2_m.ply", *cad_bunny);
		//pcl::io::loadPLYFile("bone-chisel-ciseau-a-os-m.ply", *cad_bunny);
		//pcl::io::loadPLYFile("chisel_bend.ply", *cad_bunny);
		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		transform_1(1, 1) = -1;
		pcl::transformPointCloud(*cad_bunny, *cad_bunny, transform_1);

		// ---------------- downsampling -------------------------------
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		const float leaf = 0.002f;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(cad_bunny);
		grid.filter(*cad_bunny);
		//grid.setInputCloud(above_plane);
		//grid.filter(*above_plane);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cad_bunny_update(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cad_bunny, *cad_bunny_update);

		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		
		// ========================= initialization of ICP============================================
		if (!isFirst)
		{
			// set pose to last frame (initial guess)
			pose.block<3, 1>(0, 3) = Eigen::Vector3f(initial_guess[4], initial_guess[5], initial_guess[6]);
			Eigen::Quaternionf q(initial_guess[0], initial_guess[1], initial_guess[2], initial_guess[3]);
			pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

		}
		else
		{
			// initial alignment
			PointCloudT::Ptr scene(new PointCloudT);
			PointCloudT::Ptr object(new PointCloudT);

			pcl::copyPointCloud(*above_plane, *scene);
			pcl::copyPointCloud(*cad_bunny, *object);
			Eigen::Matrix4f transformation;
		
			if (initialGuess(object, scene, transformation) == 0)
			{
				pose = transformation;
			}
			
			Eigen::Vector3d u(0, -1, 0);
			Eigen::Vector3d v(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2]);
			Eigen::Vector3d temp = u.cross(v);
			double quat_w = u.norm()*v.norm() + u.dot(v);
			Eigen::Quaternionf quat(quat_w, temp[0], temp[1], temp[2]);
			quat.normalize();

			pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
		}
		// update cad_bunny
		pcl::transformPointCloud(*cad_bunny_update, *cad_bunny_update, pose);
	//=========================================== icp ===========================================================================
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		
		icp.setInputCloud(cad_bunny_update);
		icp.setInputTarget(above_plane);

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(0.2); // suppose to be 0.1
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(10);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(1e-11);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(1e-5);
		//icp.setRANSACOutlierRejectionThreshold(0.001);

		pcl::PointCloud<pcl::PointXYZ> Final;

		for (size_t i = 0; i < 5; i++)
		{
			//Eigen::Matrix4f tempPose = pose;
			icp.align(Final);
			
			//viewer->addText(std::to_string(score) , 20, 20, "score");

			Eigen::Matrix4f d_pose = icp.getFinalTransformation();
			//pcl::transformPointCloud(*cad_bunny, *cad_bunny, d_pose);
			pose = d_pose * pose;
			
			Eigen::Vector3f u = -pose.block<3,1>(0,1);
			Eigen::Vector3f v(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2]);
			Eigen::Vector3f temp = u.cross(v);
			double quat_w = u.norm()*v.norm() + u.dot(v);
			Eigen::Quaternionf quat(quat_w, temp[0], temp[1], temp[2]);
			quat.normalize();
			Eigen::Matrix4f d_pose_2 = Eigen::Matrix4f::Identity();
			d_pose_2.block<3, 3>(0, 0) = quat.toRotationMatrix();
			Eigen::Matrix4f d_pose_2_trans = Eigen::Matrix4f::Identity();
			d_pose_2_trans(1, 3) = -0.025;
			d_pose_2 = d_pose_2_trans * d_pose_2 * d_pose_2_trans.inverse();

			pose = pose * d_pose_2;
			pcl::transformPointCloud(*cad_bunny, *cad_bunny_update, pose);
		}
		icp.setMaximumIterations(150);
		icp.align(Final);
		float score = icp.getFitnessScore();
		Eigen::Matrix4f d_pose = icp.getFinalTransformation();
		pose = d_pose * pose;

		pcl::transformPointCloud(*cad_bunny, *cad_bunny_update, pose);

		//=============================================paint cad model ====================================================================
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> bunny_color(cad_bunny_update, 0,20, 240);
		//viewer->addPointCloud(cad_bunny_update, bunny_color, "bunny");
		//viewer->addCoordinateSystem(0.2);
		
		//pcl::io::savePLYFile("bunny_cloud.ply", *pc_color);
		float* output_pose = new float[16];
		for (size_t i = 0; i < 15; i++)
		{
			output_pose[i] = i;
		}
		Eigen::Quaternionf output_quat(pose.block<3, 3>(0, 0));
		
		output_pose[8] = output_quat.w();
		output_pose[9] = output_quat.x();
		output_pose[10] = output_quat.y();
		output_pose[11] = output_quat.z();
		output_pose[12] = pose(0, 3);
		output_pose[13] = pose(1, 3);
		output_pose[14] = pose(2, 3);
		output_pose[15] = score;
		/*string pose_str = "pose: ";
		for (size_t i = 8; i < 15; i++)
		{
			pose_str = pose_str + std::to_string(output_pose[i]) + " | ";
		}*/
		//viewer->addText(pose_str,0,40,"pose");

		return output_pose;
	}
}