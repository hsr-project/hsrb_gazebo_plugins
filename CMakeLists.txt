# This package has no tests
cmake_minimum_required(VERSION 2.8.3)
project(hsrb_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    gazebo_ros
    gazebo_ros_control
    gazebo_plugins
    control_toolbox
    controller_manager
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    trajectory_msgs
    control_msgs
    tmc_control_msgs
    tmc_suction
    angles
    camera_info_manager
    image_transport
    dynamic_reconfigure
)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

generate_dynamic_reconfigure_options(
  thirdparty/cfg/CameraSynchronizer.cfg
  thirdparty/cfg/GazeboRosCamera.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        actionlib
        camera_info_manager
        gazebo_ros
        gazebo_ros_control
        gazebo_plugins
        control_toolbox
        controller_manager
        hardware_interface
        transmission_interface
        joint_limits_interface
        urdf
        trajectory_msgs
        control_msgs
        tmc_control_msgs
        tmc_suction
        image_transport
        dynamic_reconfigure
    DEPENDS gazebo
    LIBRARIES
        tmc_gazebo_hrh_gripper
        hsrb_grasp_hack
        hsrb_gazebo_ros_bumper
        hsrb_gazebo_ros_control
        hsrb_gazebo_ros_camera_utils
        hsrb_gazebo_ros_camera
        hsrb_gazebo_ros_depth_camera
        hsrb_MultiCameraPlugin
)

if(CMAKE_VERSION VERSION_LESS "3.1")
  add_compile_options(-std=c++11)
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

include_directories(
    thirdparty/include/
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

#catkin_lint: ignore link_directory
link_directories(${GAZEBO_LIBRARY_DIRS})  # This is required to provide Gazebo library path(s).

add_library(${PROJECT_NAME}_gazebo_hrh_gripper src/${PROJECT_NAME}/gazebo_hrh_gripper.cpp)
add_dependencies(${PROJECT_NAME}_gazebo_hrh_gripper tmc_control_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_gazebo_hrh_gripper ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gazebo_hrh_gripper PROPERTIES OUTPUT_NAME tmc_gazebo_hrh_gripper)

add_library(${PROJECT_NAME}_hsrb_grasp_hack src/${PROJECT_NAME}/hsrb_grasp_hack.cpp)
add_dependencies(${PROJECT_NAME}_hsrb_grasp_hack tmc_suction_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_hsrb_grasp_hack ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_grasp_hack PROPERTIES OUTPUT_NAME hsrb_grasp_hack)

add_library(${PROJECT_NAME}_hsrb_bumper src/${PROJECT_NAME}/hsrb_bumper.cpp)
target_link_libraries(${PROJECT_NAME}_hsrb_bumper ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_bumper PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_bumper)

add_library(${PROJECT_NAME}_hsrb_gazebo_ros_control src/${PROJECT_NAME}/hsrb_gazebo_ros_control_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_hsrb_gazebo_ros_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_gazebo_ros_control PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_control)

## Plugins gazebo 2.4.13 のupdateはgazebo2では古いフレームが使われるので、2.
add_library(${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils thirdparty/src/gazebo_ros_camera_utils.cpp)
add_dependencies(${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_camera_utils)

add_library(${PROJECT_NAME}_hsrb_MultiCameraPlugin thirdparty/src/MultiCameraPlugin.cpp)
target_link_libraries(${PROJECT_NAME}_hsrb_MultiCameraPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_MultiCameraPlugin PROPERTIES OUTPUT_NAME hsrb_MultiCameraPlugin)

add_library(${PROJECT_NAME}_hsrb_gazebo_ros_camera thirdparty/src/gazebo_ros_camera.cpp)
add_dependencies(${PROJECT_NAME}_hsrb_gazebo_ros_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_hsrb_gazebo_ros_camera ${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_gazebo_ros_camera PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_camera)

add_library(${PROJECT_NAME}_hsrb_gazebo_ros_depth_camera thirdparty/src/gazebo_ros_depth_camera.cpp)
add_dependencies(${PROJECT_NAME}_hsrb_gazebo_ros_depth_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_hsrb_gazebo_ros_depth_camera ${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_gazebo_ros_depth_camera PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_depth_camera)

add_library(${PROJECT_NAME}_hsrb_gazebo_ros_openni_kinect thirdparty/src/gazebo_ros_openni_kinect.cpp)
add_dependencies(${PROJECT_NAME}_hsrb_gazebo_ros_openni_kinect ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_hsrb_gazebo_ros_openni_kinect ${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_hsrb_gazebo_ros_openni_kinect PROPERTIES OUTPUT_NAME hsrb_gazebo_ros_openni_kinect)

install(
    TARGETS ${PROJECT_NAME}_gazebo_hrh_gripper
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS ${PROJECT_NAME}_hsrb_grasp_hack
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 install(
    TARGETS ${PROJECT_NAME}_hsrb_bumper
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS ${PROJECT_NAME}_hsrb_gazebo_ros_control
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
    ${PROJECT_NAME}_hsrb_gazebo_ros_camera_utils
    ${PROJECT_NAME}_hsrb_MultiCameraPlugin
    ${PROJECT_NAME}_hsrb_gazebo_ros_camera
    ${PROJECT_NAME}_hsrb_gazebo_ros_depth_camera
    ${PROJECT_NAME}_hsrb_gazebo_ros_openni_kinect
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# gazeboを利用するテストは、性能の良いGPUの入ったPCでないと動かないため、
# jenkins用サーバ機が新しくなるまで一時的にコメントアウトしておきます。
# 2015-10-23 kazuto_murase_aa

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #add_rostest_gtest(
  #  hsrb_gazebo_plugins_test
  #  test/hsrb_gazebo_plugins-test.test
  #  test/hsrb_gazebo_plugins-test.cpp
  #)
  #target_link_libraries(hsrb_gazebo_plugins_test
  #  ${catkin_LIBRARIES}
  #)
  add_rostest_gtest(
    hsrb_bumper_plugin_test
    test/hsrb_bumper_plugin-test.test
    test/hsrb_bumper_plugin-test.cpp
  )
  target_link_libraries(hsrb_bumper_plugin_test
    ${catkin_LIBRARIES}
  )
endif()

