cmake_minimum_required(VERSION 2.8.3)
project(ackermann_controller)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    urdf
    realtime_tools
    tf
    nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ackermann_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ackermann_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest std_srvs controller_manager tf)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(diffbot test/diffbot.cpp)
  target_link_libraries(diffbot ${catkin_LIBRARIES})

  add_executable(skidsteerbot test/skidsteerbot.cpp)
  target_link_libraries(skidsteerbot ${catkin_LIBRARIES})

  add_dependencies(tests diffbot skidsteerbot)

  add_rostest_gtest(ackermann_test
    test/ackermann_controller.test
    test/ackermann_test.cpp)
  target_link_libraries(ackermann_test ${catkin_LIBRARIES})
  add_rostest_gtest(ackermann_nan_test
    test/ackermann_controller_nan.test
    test/ackermann_nan_test.cpp)
  target_link_libraries(ackermann_nan_test ${catkin_LIBRARIES})
  add_rostest_gtest(ackermann_limits_test
    test/ackermann_controller_limits.test
    test/ackermann_limits_test.cpp)
  target_link_libraries(ackermann_limits_test ${catkin_LIBRARIES})
  add_rostest_gtest(ackermann_timeout_test
    test/ackermann_timeout.test
    test/ackermann_timeout_test.cpp)
  target_link_libraries(ackermann_timeout_test ${catkin_LIBRARIES})
  add_rostest(test/ackermann_multipliers.test)
  add_rostest_gtest(ackermann_fail_test
    test/ackermann_wrong.test
    test/ackermann_fail_test.cpp)
  target_link_libraries(ackermann_fail_test ${catkin_LIBRARIES})
  add_rostest_gtest(ackermann_odom_tf_test
    test/ackermann_odom_tf.test
    test/ackermann_odom_tf_test.cpp)
  target_link_libraries(ackermann_odom_tf_test ${catkin_LIBRARIES})
  add_rostest(test/ackermann_open_loop.test)
  add_rostest(test/skid_steer_controller.test)
  add_rostest(test/skid_steer_no_wheels.test)
  add_rostest(test/ackermann_radius_param.test)
  add_rostest(test/ackermann_radius_param_fail.test)
  add_rostest(test/ackermann_separation_param.test)
  add_rostest(test/ackermann_bad_urdf.test)
endif()
