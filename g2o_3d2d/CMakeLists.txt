cmake_minimum_required( VERSION 2.8 )
project(g2o_3d2d)

# 添加c++ 11标准支持
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# OpenCV
find_package( OpenCV REQUIRED )
find_package( CSparse REQUIRED )
include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)



add_executable(g2o_3d2d BA-3Dto2D.cpp )

target_link_libraries(g2o_3d2d
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )


