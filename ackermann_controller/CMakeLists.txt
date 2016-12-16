cmake_minimum_required(VERSION 2.8.3)
project(ackermann_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    nav_msgs
    ackermann_msgs
    realtime_tools
    tf
    urdf_vehicle_kinematic)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
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

catkin_install_python(PROGRAMS scripts/teleop_ackermann_joy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES ackermann_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest std_srvs controller_manager tf)

  add_executable(ackermann test/src/ackermann.cpp)
  target_link_libraries(ackermann ${catkin_LIBRARIES})

  add_dependencies(tests ackermann)

  add_rostest_gtest(ackermann_controller_test
                    test/ackermann_controller.test
                    test/src/ackermann_test.cpp)
  target_link_libraries(ackermann_controller_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_wrong_config_test
                    test/ackermann_wrong_config.test
                    test/src/ackermann_wrong_config.cpp)
  target_link_libraries(ackermann_wrong_config_test ${catkin_LIBRARIES})
  #add_rostest(test/ackermann_radius_param.test)

endif()
