cmake_minimum_required(VERSION 2.8.3)
project(glove_and_phasespace_acquisition)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  glove_acquisition
  phasespace_acquisition
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    glove_acquisition
    phasespace_acquisition
)

set(BIN_MAJOR major_listener)
set(BIN_RANDOM random_objects_listener)

# Major Listener (both Glove and PhaseSpace)
add_executable(${BIN_MAJOR}
  src/major_listener.cpp
)
target_link_libraries(${BIN_MAJOR}
  ${catkin_LIBRARIES}
)

# Random Object Listener (both Glove and PhaseSpace with objects list provided)
add_executable(${BIN_RANDOM}
  src/random_objects_listener.cpp
  src/subject.cpp
  src/item.cpp
)
target_link_libraries(${BIN_RANDOM}
  ${catkin_LIBRARIES}
)
