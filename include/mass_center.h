
#pragma once

#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

Eigen::Vector3f centar_mase_prepreke(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr(cloud);
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	//std::vector <float> moment_of_inertia;
	/*std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;*/
	Eigen::Vector3f mass_center;

	//feature_extractor.getMomentOfInertia(moment_of_inertia);
	//feature_extractor.getEccentricity(eccentricity);
	//feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	//feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	//feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	//feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	return mass_center;
}