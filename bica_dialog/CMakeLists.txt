cmake_minimum_required(VERSION 2.8.3)
project(bica_dialog)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dialogflow_ros
  roslint
  std_srvs
  sound_play
  actionlib
  actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES bica_dialog
	CATKIN_DEPENDS roscpp dialogflow_ros actionlib_msgs
	DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references,-whitespace/braces,-build/c++11")
roslint_cpp()

add_library(bica_dialog
	src/DialogInterface.cpp
)

add_dependencies(bica_dialog ${catkin_EXPORTED_TARGETS})
target_link_libraries(bica_dialog ${catkin_LIBRARIES})

install(TARGETS bica_dialog bica_dialog
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test test/test_DialogInterface.launch test/test_DialogInterface.cpp)
  target_link_libraries(
    ${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    dialog_interface_lib
  )
endif()
