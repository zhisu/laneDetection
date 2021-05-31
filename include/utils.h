/*
 * utils.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <string>

using namespace std;

vector<float>
getXLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

vector<float>
getYLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr
select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr &inliers);

pcl::PointCloud<pcl::PointXYZI>::Ptr
select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &inliers);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string fieldname, float lb, float ub);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>&, string fieldname, float lb, float ub);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& , float, float = 1.0);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& , vector<int>&, float, float = 1.0);

vector<int> indexMapping(vector<int>&, vector<int>&);

float mean(vector<float>&);
float maxvalue(vector<float>&);
float stdv(vector<float>&);


#endif /* INCLUDE_UTILS_H_ */
