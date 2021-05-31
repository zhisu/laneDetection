cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

find_package(PCL 1.8 REQUIRED)

# cmake -DCMAKE_BUILD_TYPE=Debug ../

# cmake -DUSE_DEBUG=ON ..
option(USE_DEBUG "Enter debug mode" OFF)
if (USE_DEBUG)
  add_definitions(-DDEBUG)
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB CPP_SOURCES src/*.cpp)

add_library( lanedetection ${CPP_SOURCES})
#target_compile_options(lanedetection PRIVATE -O3)
target_include_directories(lanedetection PUBLIC include/ src/)
target_link_libraries( lanedetection ${PCL_LIBRARIES})

# Test #########################################
# test plane model fitting
add_executable (test_pcfitplane test/test_pcfitplane.cpp)
target_include_directories(test_pcfitplane PUBLIC include/)
target_link_libraries (test_pcfitplane lanedetection ${PCL_LIBRARIES})

# test DBSCAN
add_executable (test_dbscan test/test_dbscan.cpp)
target_include_directories(test_dbscan PUBLIC include/)
target_link_libraries (test_dbscan lanedetection ${PCL_LIBRARIES})

# test pcshow
add_executable (test_pcshow test/test_pcshow.cpp)
target_include_directories(test_pcshow PUBLIC include/)
target_link_libraries (test_pcshow lanedetection ${PCL_LIBRARIES})

# test lane detection
add_executable (test_lanedetection test/test_lanedetection.cpp)
target_include_directories(test_lanedetection PUBLIC include/)
target_link_libraries (test_lanedetection lanedetection ${PCL_LIBRARIES})

# test utils
add_executable (test_utils test/test_utils.cpp)
target_include_directories(test_utils PUBLIC include/)
target_link_libraries (test_utils lanedetection ${PCL_LIBRARIES})

# fit plane model
#add_executable (pcfitplane src/pcfitplane.cpp)
#target_link_libraries (pcfitplane ${PCL_LIBRARIES})

#add_executable (dbscan src/dbscan.cpp)
#target_link_libraries (dbscan ${PCL_LIBRARIES})

# Application lanedet
add_executable (lanedet examples/lanedet.cpp)
target_include_directories(lanedet PUBLIC include/)
target_link_libraries (lanedet lanedetection ${PCL_LIBRARIES})


