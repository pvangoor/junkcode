project(make_depth_maps)
cmake_minimum_required(VERSION 3.0)
# set(CMAKE_CXX_STANDARD 17)


find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( TBB REQUIRED )


add_executable(make_depth_maps main.cpp)



target_include_directories(make_depth_maps
PRIVATE ${OPENCV_INCLUDE_DIRS}
PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(make_depth_maps
PRIVATE cxx_std_17
)

target_link_libraries(make_depth_maps
    ${OpenCV_LIBS}
    ${TBB_IMPORTED_TARGETS}
)