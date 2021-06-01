/*
 * lanepar.cpp
 *
 *  Created on: Jun 1, 2021
 *      Author: aina
 */

#include "lanepar.h"

void
LanePar::parseLine(string& line){
  stringstream ss(line);
  string key;
  string value;
  getline(ss, key, ' ');
  while(getline(ss, value, ' ')){
      if(!value.empty()) break;
  }
  if(key=="SUBREGION_WIDTH")
      subregion_width = stof(value);
  if(key=="PLANE_DIST_THRESHOLD")
      plane_dist_threshold = stof(value);
  if(key=="LANE_WIDTH")
      lane_width = stof(value);
  if(key=="DBSCAN_MINPTS")
      dbscan_minpts = stoi(value);
  if(key=="LANEMARK_MINPTS")
      lanemark_minpts = stoi(value);
}

void
LanePar::parseParfile(string parfile){
  std::ifstream cFile (parfile);
  if (cFile.is_open())
  {
      std::string line;
      while(getline(cFile, line))
     {
          if( line.empty() || line[0] == '#' )
          {
              continue;
          }
          parseLine(line);
      }
  }
  else
  {
      std::cerr << "Couldn't open config file for reading.\n";
  }
}


