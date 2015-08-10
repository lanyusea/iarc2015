cmake_minimum_required(VERSION 2.8.3)
project(ukftest)



## System dependencies are found with CMake's conventions
set(Boost_USE_STATIC_LIBS ON)
find_package( Boost COMPONENTS iostreams system filesystem REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})

## Opencv
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIBRARY_DIR})
link_libraries(${OpenCV_LIBS})

set(EIGEN_NO_DEBUG OFF)
##find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

## find Yu Yun math library
file(GLOB UAV_MATH_LIB_SOURCES src/math_lib/*.c)
include_directories("${PROJECT_SOURCE_DIR}/src/math_lib")
SET_SOURCE_FILES_PROPERTIES(${UAV_MATH_LIB_SOURCES} PROPERTIES LANGUAGE CXX)

## find my marker lib
file(GLOB MY_MARKER_LIB_SOURCES src/marker_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/src/marker_lib")

## find UKF lib
file(GLOB MY_SRUKF_LIB_SOURCES src/SRUKF_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/src/SRUKF_lib")

## find pid lib
file(GLOB MY_PID_LIB_SOURCES src/PID_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/src/PID_lib")

## find T lib
file(GLOB MY_T_LIB_SOURCES src/T_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/src/T_lib")

## find T lib
file(GLOB MY_CT_LIB_SOURCES src/CT_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/src/CT_lib")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  trackInfo.msg
  ukfData.msg
  UAV.msg
  markerInfo.msg
  Steer.msg
  laserPoint.msg
  avoidCtrl.msg
  laserInfo.msg
  trackCtrl.msg
  randomCtrl.msg
  Ctrl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ukftest
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  message_runtime
  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(odom_estimation_node src/odom_estimation_node.cpp src/odom_estimation.cpp src/SRUKF.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(odom_estimation_node ukftest_generate_messages_cpp)
target_link_libraries(odom_estimation_node ${catkin_LIBRARIES})

add_executable(simulate_imuvo_node src/simulate_imuvo_node)
add_dependencies(simulate_imuvo_node ukftest_generate_messages_cpp)
target_link_libraries(simulate_imuvo_node ${catkin_LIBRARIES})

add_executable(uav_controller_node src/uav_controller.cpp src/pid.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_controller_node ${catkin_LIBRARIES})

add_executable(neg_tracker_test src/NEGtracker.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(neg_tracker_test ukftest_generate_messages_cpp)
target_link_libraries(neg_tracker_test ${catkin_LIBRARIES})

add_executable(neg_marker_test_node src/neg_marker_test_node.cpp ${MY_MARKER_LIB_SOURCES})
add_dependencies(neg_marker_test_node ukftest_generate_messages_cpp)
target_link_libraries(neg_marker_test_node ${catkin_LIBRARIES})

add_executable(neg_estimation_node src/NEG/neg_estimation_node src/NEG/neg_estimation.cpp ${MY_SRUKF_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(neg_estimation_node ukftest_generate_messages_cpp)
target_link_libraries(neg_estimation_node ${catkin_LIBRARIES})

add_executable(uav_neg_controller_node src/NEG/uav_neg_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_neg_controller_node ${catkin_LIBRARIES})

add_executable(uav_controller_noyaw_node src/uav_controller_noyaw.cpp src/pid.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_controller_noyaw_node ukftest_generate_messages_cpp)
target_link_libraries(uav_controller_noyaw_node ${catkin_LIBRARIES})

add_executable(neg_T_test_node src/neg_T_test_node.cpp ${MY_T_LIB_SOURCES})
add_dependencies(neg_T_test_node ukftest_generate_messages_cpp)
target_link_libraries(neg_T_test_node ${catkin_LIBRARIES})

add_executable(neg_cttracker_test_node src/NEG/neg_cttracker_test_node.cpp ${MY_CT_LIB_SOURCES})
add_dependencies(neg_cttracker_test_node ukftest_generate_messages_cpp)
target_link_libraries(neg_cttracker_test_node ${catkin_LIBRARIES})

add_executable(houghcircle_test_node src/houghcircle/houghcircle.cpp)
add_dependencies(houghcircle_test_node ukftest_generate_messages_cpp)
target_link_libraries(houghcircle_test_node ${catkin_LIBRARIES})

add_executable(image_reader_test_node src/color_tracker/image_reader_test_node.cpp)
add_dependencies(image_reader_test_node ukftest_generate_messages_cpp)
target_link_libraries(image_reader_test_node ${catkin_LIBRARIES})

add_executable(color_tracking_onxu_test_node src/color_tracker/color_tracking_onxu_test_node.cpp)
add_dependencies(color_tracking_onxu_test_node ukftest_generate_messages_cpp)
target_link_libraries(color_tracking_onxu_test_node ${catkin_LIBRARIES})

add_executable(my_vfh_test_node src/laser_test/my_vfh_test_node.cpp src/laser_test/OpenRadar.cpp)
add_dependencies(my_vfh_test_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_test_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(laser_neg_estimation_node src/laser_avoid/laser_neg_estimation_node src/laser_avoid/laser_neg_estimation.cpp ${MY_SRUKF_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(laser_neg_estimation_node ukftest_generate_messages_cpp)
target_link_libraries(laser_neg_estimation_node ${catkin_LIBRARIES})

add_executable(uav_neg_laser_controller_node src/laser_avoid/uav_neg_laser_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_neg_laser_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_neg_laser_controller_node ${catkin_LIBRARIES})

add_executable(uav_b_laser_controller_node src/laser_avoid/uav_b_laser_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_b_laser_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_b_laser_controller_node ${catkin_LIBRARIES})

add_executable(my_vfh_node src/laser_test/my_laserRadar/my_vfh_node.cpp src/laser_test/my_laserRadar/myRadar.cpp)
add_dependencies(my_vfh_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(my_vfh_v2_node src/laser_test/laser_radar_node/my_vfh_node.cpp src/laser_test/laser_radar_node/myRadar.cpp)
add_dependencies(my_vfh_v2_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_v2_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(my_vfh_4rp_node src/laser_test/laser_radar_4rp_node/my_vfh_node.cpp src/laser_test/laser_radar_4rp_node/myRadar.cpp)
add_dependencies(my_vfh_4rp_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_4rp_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(my_vfh_4rp_4acc_node src/laser_test/laser_radar_4rp_4acc_node/my_vfh_node.cpp src/laser_test/laser_radar_4rp_4acc_node/myRadar.cpp)
add_dependencies(my_vfh_4rp_4acc_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_4rp_4acc_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(my_vfh_4rp_4acc_unsmooth_node src/laser_test/laser_radar_4rp_4acc_unsmooth_node/my_vfh_node.cpp src/laser_test/laser_radar_4rp_4acc_unsmooth_node/myRadar.cpp)
add_dependencies(my_vfh_4rp_4acc_unsmooth_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_4rp_4acc_unsmooth_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(my_vfh_4rp_4v_node src/laser_test/laser_radar_4rp_4v1.5_node/my_vfh_node.cpp src/laser_test/laser_radar_4rp_4v1.5_node/myRadar.cpp)
add_dependencies(my_vfh_4rp_4v_node ukftest_generate_messages_cpp)
target_link_libraries(my_vfh_4rp_4v_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(uav_avoid_neg_controller_node src/state_ctrl/uav_avoid_neg_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_avoid_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_avoid_neg_controller_node ${catkin_LIBRARIES})


add_executable(uav_track_neg_controller_node src/state_ctrl/uav_track_neg_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_track_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_track_neg_controller_node ${catkin_LIBRARIES})

add_executable(uav_random_neg_controller_node src/state_ctrl/uav_random_neg_controller.cpp)
add_dependencies(uav_random_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_random_neg_controller_node ${catkin_LIBRARIES})

add_executable(uav_random_local_controller_node src/state_ctrl/uav_random_local_controller.cpp)
add_dependencies(uav_random_local_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_random_local_controller_node ${catkin_LIBRARIES})

add_executable(uav_boundary_local_controller_node src/state_ctrl/uav_boundary_local_controller.cpp)
add_dependencies(uav_boundary_local_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_boundary_local_controller_node ${catkin_LIBRARIES})


add_executable(uav_landing_neg_controller_node src/state_ctrl/uav_landing_neg_controller.cpp)
add_dependencies(uav_landing_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_landing_neg_controller_node ${catkin_LIBRARIES})


add_executable(uav_takeoff_controller_node src/state_ctrl/uav_takeoff_controller.cpp)
add_dependencies(uav_takeoff_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_takeoff_controller_node ${catkin_LIBRARIES})

add_executable(uav_state_controller_node src/state_ctrl/uav_state_controller.cpp)
add_dependencies(uav_state_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_state_controller_node ${catkin_LIBRARIES})

add_executable(color_tracking_node src/color_tracker/color_tracking_node.cpp)
add_dependencies(color_tracking_node ukftest_generate_messages_cpp)
target_link_libraries(color_tracking_node ${catkin_LIBRARIES})

add_executable(uav_state2_controller_node src/state_ctrl/uav_state2_controller.cpp)
add_dependencies(uav_state2_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_state2_controller_node ${catkin_LIBRARIES})

add_executable(uav_takeoff2_controller_node src/state_ctrl/uav_takeoff2_controller.cpp)
add_dependencies(uav_takeoff2_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_takeoff2_controller_node ${catkin_LIBRARIES})

add_executable(reg_test_node src/testReg/reg_test_node.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(reg_test_node ukftest_generate_messages_cpp)
target_link_libraries(reg_test_node ${catkin_LIBRARIES})

add_executable(uav_tracking_controller_node src/marker_tracking/uav_tracking_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_tracking_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_tracking_controller_node ${catkin_LIBRARIES})


add_executable(uav_state_track_avoid_controller_node src/state_ctrl/uav_state_track_avoid_controller.cpp)
add_dependencies(uav_state_track_avoid_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_state_track_avoid_controller_node ${catkin_LIBRARIES})

add_executable(uav_marker_tracker_controller_node src/state_ctrl/uav_marker_tracker_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_marker_tracker_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_marker_tracker_controller_node ${catkin_LIBRARIES})


add_executable(uav_status_vicon_controller_node src/state_ctrl/uav_status_vicon_controller.cpp)
add_dependencies(uav_status_vicon_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_status_vicon_controller_node ${catkin_LIBRARIES})

add_executable(uav_avoid_neg_acc_controller_node src/state_ctrl/uav_avoid_neg_acc_controller.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_avoid_neg_acc_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_avoid_neg_acc_controller_node ${catkin_LIBRARIES})

add_executable(test_imu_node src/test_imu/test_imu.cpp ${UAV_MATH_LIB_SOURCES})
add_dependencies(test_imu_node ukftest_generate_messages_cpp)
target_link_libraries(test_imu_node ${catkin_LIBRARIES})

add_executable(uav_poticalflow_neg_controller_node src/opticalflow_test/uav_poticalflow_neg_controller.cpp ${MY_PID_LIB_SOURCES} ${UAV_MATH_LIB_SOURCES})
add_dependencies(uav_poticalflow_neg_controller_node ukftest_generate_messages_cpp)
target_link_libraries(uav_poticalflow_neg_controller_node ${catkin_LIBRARIES})

