### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(picotts)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(DEPS  roscpp  roslib  std_msgs)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
# nothing to do in this package
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# nothing to do in this package
### 5 - Invoke message/service/action generation (generate_messages())
# nothing to do in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(${catkin_INCLUDE_DIRS})
add_executable(picotts.exe src/picotts.cpp)
target_link_libraries(picotts.exe  ${catkin_LIBRARIES})
### 8 - Tests to build (catkin_add_gtest())
# nothing to do in this package
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
