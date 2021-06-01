/*
 * lanedetection.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "lanedetection.h"
#include <vector>
#include <math.h>
#include "pcshow.h"

using namespace std;



int
evalLaneCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& fullcloud, std::vector<int> &cluster, float laneW)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = select(fullcloud, cluster);
  // TODO May have a direct API for PointCloud in PCL
  vector<float> xlimits = getXLimits(cloud);
  vector<float> ylimits = getYLimits(cloud);
  int xOriented = 1; // road travel along the x Axis, 0 for y Axis otherwise
  if(abs(ylimits[1]-ylimits[0]) > abs(xlimits[1]-xlimits[0]))
    xOriented = 0;

  vector<float> binWidth;
  int nbins = 0; // check the lane width in each bin
  if(xOriented){
      float lb = xlimits[0];
      // Calculate y width, TODO Outlier handling
      nbins = ceil((xlimits[1]-xlimits[0])/laneW);
      binWidth.assign(nbins, 0.0);
      for(int i =0; i<nbins; i++){
	  float ub = lb+laneW-0.01;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr bin_pc = filterByField(cloud, "x", lb, ub);
	  vector<float> bin_ylimits = getYLimits(bin_pc);
	  binWidth[i]=bin_ylimits[1]-bin_ylimits[0];
	  lb = ub;
      }
  }else{
      float lb = ylimits[0];
      // Calculate x width, TODO Outlier handling
      nbins = ceil((ylimits[1]-ylimits[0])/laneW);
      binWidth.assign(nbins, 0.0);
      for(int i =0; i<nbins; i++){
	  float ub = lb+laneW-0.01;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr bin_pc = filterByField(cloud, "y", lb, ub);
	  vector<float> bin_xlimits = getXLimits(bin_pc);
	  binWidth[i]=bin_xlimits[1]-bin_xlimits[0];
	  lb = ub;
      }
  }

  int laneflag = 1;
  // Heuristic to check if cloud is possibly a lane mark.
  if(mean(binWidth)<3*laneW && maxvalue(binWidth)<5*laneW && stdv(binWidth)<laneW)
    laneflag=0;

  //TODO if(sum(binWidth>5*laneW)/length(binWidth)<0.1 & sum(binWidth<3*laneW)/length(binWidth)>0.8)
  //    flag = 1;
  int cnt5W=0, cnt3W=0;
  for(const auto& w: binWidth){
    if(w>5*laneW) cnt5W++;
    if(w<3*laneW) cnt3W++;
  }
  if(cnt5W<0.1*binWidth.size() && cnt3W>0.8*cnt3W){
      laneflag=0;
  }

  return laneflag;
}

std::vector<int>
findLanesByConfig(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float INTENSITY_THRESHOLD, float eps, LanePar par)
{
  int minPts = par.dbscan_minpts; // int minPts = 10;

  // Filter by intensity to get lane point
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(cloud, indsetFiltered, INTENSITY_THRESHOLD);
#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
#endif

  // clustering
  DBSCAN dbscan;
  dbscan.setInputCloud (ptc);
  std::vector<int> dbclustering = dbscan.segment (eps, minPts);
  std::vector<std::vector<int>> & clusters = dbscan.clusters;

#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
  int tmp_cnt=0;
  for(const auto & p: dbclustering){
      if(p>=0) tmp_cnt++;
  }
  std::cout<<"In findLanesByConfig: number of inlier points tmp_cnt= "<<tmp_cnt<<std::endl;
#endif

  // detect lanes
  int lanept_cnt=0;
  float laneW = par.lane_width; // float laneW = 0.2; // float laneW = 0.15; // TODO read from config, may use 0.2
  int numClusters = clusters.size();
  for(int i=0; i<numClusters; i++){
      // filter small clusters with less than minClusterPts
      int minClusterPts = par.lanemark_minpts; // TODO read from config
      if (clusters[i].size()<minClusterPts){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  continue;
      }
      int flag = evalLaneCluster(ptc, clusters[i], laneW); // TODO ptc instead of cloud .return 0 if it's a possible lane
      if(flag){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  //DEBUG
	  // std::cout<<"cluster "<<i<<" is not a lane, clusters[i].size() = "<<clusters[i].size()<<std::endl;
	  // pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptc = select(cloud, clusters[i]);
	  // custom_pcshow(cluster_ptc);
      }
      else lanept_cnt+=clusters[i].size();
  }

#ifdef DEBUG
  std::cout<<"Find " <<lanept_cnt <<" lane points."<<std::endl;
#endif

  std::vector<int> indset;
  for(int i=0; i<dbclustering.size(); i++){
      if(dbclustering[i]>=0) {
	  indset.push_back(indsetFiltered[i]); // indset.push_back(i);
      }
  }
  // DEBUG
#ifdef DEBUG
  std::cout<<"dbclustering.size() = "<<dbclustering.size()<<", indset.size() = "<<indset.size()<<std::endl;
#endif
  return indset;
}

std::vector<int>
findLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud, LanePar par)
{

  std::vector<int> indset;
  vector<int> indsetFiltered;

#ifdef DEBUG
  std::cout<<"DEBUG is on, and visualize the point clouds for debug in findLanes." <<std::endl;
  custom_pcshow(inCloud);
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = filterByIntensity(inCloud, indsetFiltered, 0.000); // TODO
#ifdef DEBUG
  custom_pcshow(cloud);
#endif

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);
#ifdef DEBUG
  std::cout<<"numPts = "<<numPts<<std::endl;
#endif
  // find lanes with multiple configurations
  // intensity_threshold=0.1, eps = 0.2
  std::vector<int> indset1 = findLanesByConfig(cloud, 0.1, 0.2, par);
#ifdef DEBUG
  std::cout<<"indset1.size() = "<<indset1.size()<<std::endl;
  custom_pcshow(select(cloud, indset1));
#endif
  std::vector<int> indset2 = findLanesByConfig(cloud, 0.12, 0.1, par);
#ifdef DEBUG
  std::cout<<"indset2.size() = "<<indset2.size()<<std::endl;
  custom_pcshow(select(cloud, indset2));
#endif
//  std::vector<int> indset3 = findLanesByConfig(cloud, 0.05, 0.4);
//  // std::vector<int> indset3 = findLanesByConfig(cloud, 0.1, 0.4);
//  // std::vector<int> indset3 = findLanesByConfig(cloud, 0.09, 0.2);
//#ifdef DEBUG
//  std::cout<<"indset3.size() = "<<indset3.size()<<std::endl;
//  custom_pcshow(select(cloud, indset3));
//#endif

//#ifdef DEBUG
//  custom_pcshow(select(cloud, indset1));
//  custom_pcshow(select(cloud, indset2));
//  custom_pcshow(select(cloud, indset3));
//#endif


  // Combine the results together
  for(const auto & p: indset1)
    lanepts[p]=1;
  for(const auto & p: indset2)
    lanepts[p]=1;
//  for(const auto & p: indset3)
//    lanepts[p]=1;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) indset.push_back(indsetFiltered[i]); // if(lanepts[i]>0) indset.push_back(i);
  }

//  // reclustering for the laneset
//  pcl::PointCloud<pcl::PointXYZI>::Ptr laneptc = select(cloud, indset);
//  std::vector<int> indset_final = findLanesByConfig(cloud, 0.01, 0.4);
//  return indset_final;

  return indset;
}

std::vector<int>
findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string fieldname, LanePar par)
{
  pcl::PCDWriter writer;
  // get point cloud in ROI
  vector<int> indsetCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptcROI =
  filterByField(cloud, indsetCloud, fieldname, roi[0], roi[1]);

  std::cout<<"INFO: process points in range "<<fieldname<<": "<<roi[0]<<", "<<roi[1]<<std::endl;

#ifdef DEBUG
  // Write the extracted plane to disk
  std::cout<<"INFO: process points in range "<<fieldname<<": "<<roi[0]<<", "<<roi[1]<<std::endl;
  writer.write<pcl::PointXYZI> ("ptcROI_debug.pcd", *ptcROI, false);
  //custom_pcshow(ptcROI);
#endif

  // Step 1: Extract road plane
  float distThreshold = par.plane_dist_threshold; // float distThreshold = 0.05; // float distThreshold = 0.15;
  vector<int> indsetROI;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = pcfitplane(ptcROI, indsetROI, distThreshold);
  pcfitplaneByROI(ptcROI, indsetROI, distThreshold, fieldname);
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = select(ptcROI, indsetROI);

#ifdef DEBUG
  custom_pcshow(planeCloud);
#endif

//  // Step 2: Find lane marks
  std::vector<int> indsetROIinCloud = indexMapping(indsetCloud,indsetROI);

  std::vector<int> indsetPlane = findLanes(planeCloud, par);

#ifdef DEBUG
  // custom_pcshow(select(planeCloud, indsetPlane));
#endif

  std::vector<int> indset = indexMapping(indsetROIinCloud,indsetPlane);

  return indset;
}

void
findLanesInPointcloud(string pcdfile, LanePar& par){

  std::cout<<"Running lane detection for "+pcdfile<<" ......"<<std::endl;
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (pcdfile, *cloud);
  // pcl::io::loadPCDFile ("point_cloud_00007.pcd", *cloud);

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);

  float subregion_width = par.subregion_width;
  string fieldname = "x";

  vector<float> range;
  vector<float> xlimits = getXLimits(cloud);
  vector<float> ylimits = getYLimits(cloud);
  if(xlimits[1]-xlimits[0]<ylimits[1]-ylimits[0]){
      fieldname = "y";
      range=ylimits;
  }else{
      fieldname = "x";
      range=xlimits;
  }

  int N = (range[1]-range[0]+1)/subregion_width;

  vector<float> roi(2,0.0);

  for(int i=0; i<=N; i++){
      roi[0] = range[0]+subregion_width*i-10;
      roi[1] = range[0]+subregion_width*(i+1);
      if(i==N) roi[1] = range[1];
      std::vector<int> indset = findLanesByROI(cloud, roi, fieldname, par);
      pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud, indset);
      // custom_pcshow(lane_ptc);
      for(const auto & p: indset)
        lanepts[p]=1;
  }
  std::vector<int> lane_indset;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) lane_indset.push_back(i); // if(lanepts[i]>0) indset.push_back(i);
  }

  std::cout<<"Lane detection completed!"<<std::endl;

  // Write the extracted plane to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> (pcdfile+".output", *select(cloud, lane_indset), false);

#ifdef DEBUG
  custom_pcshow(select(cloud, lane_indset));
#endif

}

void
findLanesInPointcloud(string pcdfile, string parfile){
  LanePar par(parfile);
  findLanesInPointcloud(pcdfile, par);
}

void
findLanesInPointcloud(string pcdfile){
  LanePar par;
  findLanesInPointcloud(pcdfile, par);
}
