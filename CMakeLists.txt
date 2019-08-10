cmake_minimum_required(VERSION 2.8.7)
project(AllegroHand)

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)

set (CMAKE_CXX_FLAGS "-O2 -Wall ")
#-std=c++11

find_package(SAI2-MODEL)

#include Sai2-Urdf
FIND_PACKAGE(SAI2-URDF REQUIRED)

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)


include_directories(
	${SAI2-MODEL_INCLUDE_DIRS}
	${SAI2-URDF_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR})

Set (ALL_LIBS 
	${SAI2-MODEL_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	)


add_subdirectory (grasp)
