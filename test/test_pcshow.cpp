/*
 * test_pcshow.cpp
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#include "pcshow.h"
#include "utils.h"

int
main ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI (
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_filtered.pcd", *cloudXYZI);
  custom_pcshow(cloudXYZI);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = filterByIntensity(cloudXYZI, 0.15); // TODO
  // writer.write<pcl::PointXYZI> ("extracted_plane_filtered.pcd", *cloud_filtered, false);
  custom_pcshow(cloud_filtered);

  return 0;
}

