cmake_minimum_required(VERSION 2.8.3)
project(ply_mesh_to_depth_maps)

if( ${CMAKE_VERSION} VERSION_LESS "3.8.2" )
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

##################
## Dependencies ##
##################

# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Boost
find_package(Boost REQUIRED COMPONENTS
            program_options
            filesystem
            graph
            system)

#Eigen3
find_package(Eigen3 3.3.0 REQUIRED NO_MODULE)

if (Eigen3_FOUND)
  message("Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

# Sophus
# First search for a custom build of Sophus within this project
find_package(Sophus REQUIRED NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/install/)
# find_package(Sophus REQUIRED)

if (Sophus_FOUND)
  message("Found Sophus lib : ${Sophus_VERSION} - ${Sophus_DIR}")
endif (Sophus_FOUND)

# OpenCV
find_package(OpenCV 3)

if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_DIR} - ${OpenCV_LIBS}")
else (OpenCV_FOUND)
  find_package(OpenCV 4 REQUIRED)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_DIR} - ${OpenCV_LIBS}")
endif (OpenCV_FOUND)

###########
## Build ##
###########

add_executable(${PROJECT_NAME} apps/ply_mesh_to_depth_maps.cpp
              source/colmap_reader.cpp)

target_include_directories(${PROJECT_NAME}
  # PUBLIC
  PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/source
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/thirdparty/rayint_libs
    ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  # PRIVATE
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${OpenCV_LIBS}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
  # INTERFACE
    Eigen3::Eigen
    Sophus::Sophus
)
