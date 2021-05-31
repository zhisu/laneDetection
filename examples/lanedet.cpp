/*
 * lanedet.cpp
 *
 *  Created on: May 30, 2021
 *      Author: aina
 */

#include "lanedetection.h"


int main(int argc, char* argv[])
{
  string pcdfile;
  if(argc>=2){
      pcdfile = argv[1];
  }else{
    std::cout<<"Usage: lanedet pcdfile \n"<<std::endl;
    std::cout<<"Please provide a point cloud input file in pcd format."<<std::endl;
    std::cout<<"Exit without lane detection. \n"<<std::endl;
    return 1;
  }
  findLanesInPointcloud(pcdfile);
  return (0);
}


