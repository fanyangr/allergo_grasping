cmake_minimum_required(VERSION 2.8.7)
project(AllegroHand)

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)
# SET(CMAKE_CXX_FLAGS "-std=c++98 -Wno-error=c++11-extensions -I/usr/include -I/usr/local/include")
set (CMAKE_CXX_FLAGS "-Dlinux -std=c++11 -O2")

find_package(SAI2-MODEL)

#include Sai2-Urdf
FIND_PACKAGE(SAI2-URDF REQUIRED)

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)

# - hiredis
 #find_library(HIREDIS_LIBRARY hiredis)
# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

include_directories(
	${SAI2-MODEL_INCLUDE_DIRS}
	${SAI2-URDF_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	#${HIREDIS_LIBRARY}
	#${SAI2-COMMON_LIBRARIES}
)

Set (ALL_LIBS 
	${SAI2-MODEL_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	#${HIREDIS_LIBRARY}	
	#${SAI2-COMMON_LIBRARIES}
	)


add_subdirectory (grasp)
