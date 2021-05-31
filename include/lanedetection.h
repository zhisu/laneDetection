/*
 * lanedetection.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_LANEDETECTION_H_
#define INCLUDE_LANEDETECTION_H_

#include <vector>
#include <pcl/point_cloud.h>

#include "pcfitplane.h"
#include "dbscan.h"
#include "utils.h"

void
findLanesInPointcloud(string pcdfile);

std::vector<int>
findLanesByConfig(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float INTENSITY_THRESHOLD, float eps);

std::vector<int>
findLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

std::vector<int>
findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string="x");

int evalLaneCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &cluster, float laneW);

#endif /* INCLUDE_LANEDETECTION_H_ */
