cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_floor_filter)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure
                                        geometry_msgs
                                        roscpp
                                        tf
                                        tf2_ros
                                        tf_conversions
                                        sensor_msgs
                                        image_transport
                                        pcl_ros
                                        
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )

generate_dynamic_reconfigure_options(cfg/FloorFilter.cfg)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS dynamic_reconfigure
                   geometry_msgs
                   roscpp
                   tf
                   tf2_ros
                   tf_conversions
                   sensor_msgs
                   image_transport
                   pcl_ros
                   
  )
                                        

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${Eigen_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${Eigen_LIBRARY_DIRS}) 

add_library(pointcloud_floor_filter_lib src/depth_to_pointcloud.cpp
                                        src/floor_plane_estimation.cpp
                                        src/pointcloud_floor_filter_lib.cpp
)
target_link_libraries(pointcloud_floor_filter_lib ${catkin_LIBRARIES}
                                                  ${Boost_LIBRARIES}
                                                  ${Eigen_LIBRARIES})
add_dependencies(pointcloud_floor_filter_lib ${PROJECT_NAME}_gencfg)

add_executable(pointcloud_floor_filter_node ros/pointcloud_floor_filter.cpp)
target_link_libraries(pointcloud_floor_filter_node pointcloud_floor_filter_lib ${Boost_LIBRARY_DIRS} ${catkin_LIBRARIES})
add_dependencies(pointcloud_floor_filter_node ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pointcloud_floor_filter_node pointcloud_floor_filter_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

