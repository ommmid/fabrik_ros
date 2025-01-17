cmake_minimum_required(VERSION 3.5)
project(fabrik_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
  interactive_markers
  rviz_visual_tools
  roscpp
  urdf
)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
#   CATKIN_DEPENDS
  DEPENDS
    EIGEN3

)

message([WARNING] "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
message([WARNING] "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# find_package looks for .so and header files (basically it does find_path and find_library combined)
# we need to find the package by CONFIG mode because in fabrik library, I create fabrikConfig.cmake
# if fabrik had a FindFabrik.config instead, I would use find_package in MODULE mode
find_package(fabrik CONFIG REQUIRED)
# if package is found then fabrik_FOUND is set to true
# Config mode attempts to locate a configuration file provided by the package to be found.
# A cache entry called fabrik_DIR is created to hold the directory containing the config file
# The full path to the configuration file is stored in the cmake variable <package>_CONFIG

if(${fabrik_FOUND})
  message([WARNING] "----------------------fabrik was found. Yayy!")
else()
  message([WARNING] "----------------------fabrik was not found. Damn")
endif()

message([WARNING] " $fabrik_INCLUDE_DIR: ${FABRIK_INCLUDE_DIR}")
message([WARNING] " $fabrik_INCLUDE_DIRS: ${FABRIK_INCLUDE_DIRs}")

find_package(ccd CONFIG REQUIRED)

if(${ccd_FOUND})
  message([WARNING] "----------------------ccd was found. Yayy!")
else()
  message([WARNING] "----------------------ccd was not found. Damn")
endif()

message([WARNING] " $ccd_INCLUDE_DIR: ${CCD_INCLUDE_DIR}")
message([WARNING] " $ccd_INCLUDE_DIRS: ${CCD_INCLUDE_DIRs}")


include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${FABRIK_INCLUDE_DIR}
                    ${CCD_INCLUDE_DIR})


# fabrik
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node fabrik ${catkin_LIBRARIES} )

add_executable(rviz_visual_tools_demo src/rviz_visual_tools_demo.cpp)
target_link_libraries(rviz_visual_tools_demo ${catkin_LIBRARIES} )

add_executable(visualize_fabrik_solution_2D src/visualize_fabrik/visualize_fabrik_solution_2D.cpp)
target_link_libraries(visualize_fabrik_solution_2D fabrik ${catkin_LIBRARIES} )

add_executable(visualize_fabrik_solution_3D src/visualize_fabrik/visualize_fabrik_solution_3D.cpp)
target_link_libraries(visualize_fabrik_solution_3D fabrik ${catkin_LIBRARIES} )

add_executable(test_calculateReach src/test_calculateReach.cpp)
target_link_libraries(test_calculateReach fabrik ${catkin_LIBRARIES} )

include_directories(include)
add_library(FabrikModel src/fabrik_model/fabrik_model.cpp)
target_link_libraries(FabrikModel fabrik)

add_executable(urdf2fabrik src/fabrik_model/convert_urdf_to_fabrik_model.cpp)
target_link_libraries(urdf2fabrik fabrik FabrikModel  ${catkin_LIBRARIES} )


#  ccd
# add_executable(visualize_ccd_solution_2D src/visualize_fabrik/visualize_ccd_solution_2D.cpp)
# target_link_libraries(visualize_ccd_solution_2D ccd  ${catkin_LIBRARIES})

