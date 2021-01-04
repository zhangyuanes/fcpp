cmake_minimum_required(VERSION 3.10)
project(fcpp)

set(CMAKE_CXX_STANDARD 14)

#find_package (Eigen3 REQUIRED)
#find_package(OpenCV 3 REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)
#include_directories(OpenCV_INCLUDE_DIRS)
# 头文件
include_directories(${PROJECT_SOURCE_DIR}/include/
#                    ${EIGEN3_INCLUDE_DIR}
#                    ${OpenCV_INCLUDE_DIRS}
)

# 设置路径（下面生成共享库的路径）
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 即生成的共享库在工程文件夹下的lib文件夹中

add_library(Planner_shared
           #src/Coverage_path_planning.cpp
           #src/A_star.cpp
           #src/Load_map_file.cpp
           #src/Convex.cpp
)

# A*测试
#add_executable(A_star src/A_star.cpp)

# 全覆盖测试
#add_executable(ccpp_demo src/Coverage_path_planning.cpp)
#target_link_libraries(ccpp_demo ${OpenCV_LIBS})

#add_executable(ccpp src/Load_map_file.cpp)
#target_link_libraries(load_map ${OpenCV_LIBS} Planner_shared)