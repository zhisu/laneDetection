/*
 * utils.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "utils.h"

using namespace std;

vector<float>
getXLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  vector<float> xlimits(2,0.0);
  if(cloud->points.size()==0) return xlimits;
  xlimits[0]=cloud->points[0].x;
  xlimits[1]=xlimits[0];
  for(const auto & pt: cloud->points){
      if(pt.x>xlimits[1]) xlimits[1] = pt.x;
      if(pt.x<xlimits[0]) xlimits[0] = pt.x;
  }
  return xlimits;
}

vector<float>
getYLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  vector<float> ylimits(2,0.0);
  if(cloud->points.size()==0) return ylimits;
  ylimits[0]=cloud->points[0].y;
  ylimits[1]=ylimits[0];
  for(const auto & pt: cloud->points){
      if(pt.y>ylimits[1]) ylimits[1] = pt.y;
      if(pt.y<ylimits[0]) ylimits[0] = pt.y;
  }
  return ylimits;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr &inliers)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);

  // Extract the inliers
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*output);

  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &inliers)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = 0; i<inliers.size(); i++){
      output->points.push_back(cloud->points[inliers[i]]);
  }
  output->width = output->points.size();
  output->height = 1; // 1 means unorganized point cloud
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string fieldname, float lb, float ub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass(false); // Initializing with true will allow us to extract the removed indices
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldname);
  pass.setFilterLimits (lb, ub);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int> &inlierIndset, string fieldname, float lb, float ub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass(true); // Initializing with true will allow us to extract the removed indices
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldname);
  pass.setFilterLimits (lb, ub);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered); // filtered point cloud
  pass.filter (inlierIndset); // TODO filtered index array, not efficient
  return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float lb, float ub)
{
  return filterByField(cloud, "intensity", lb, ub);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, vector<int> &inlierIndset, float lb, float ub)
{
  return filterByField(cloud, inlierIndset, "intensity", lb, ub);
}

vector<int> indexMapping(vector<int>& X, vector<int>&Y)
{
  // returns X[Y]

  std::vector<int> Z;
  for(const auto & p: Y){
      Z.push_back(X[p]);
  }
  return Z;
}

float mean(vector<float>& v)
{
  if(v.empty()) return 0.0;
  float total = 0.0;
  for(auto &p: v) total+=p;
  return total/v.size();
}

float maxvalue(vector<float>& v)
{
  float r=0.0;
  for(auto &p: v){
      if(p>r) r=p;
  }
  return r;
}

float stdv(vector<float>& v)
{
  float mu = mean(v);
  float total = 0.0;
  for(const auto & p: v ){
      total = total+(p-mu)*(p-mu);
  }
  float sigma = sqrt(total/(v.size()-1.0));
  return sigma;
}



