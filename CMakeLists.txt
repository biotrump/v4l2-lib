cmake_minimum_required(VERSION 2.8)
#C and C++ APIs for V4L2 under openCV
PROJECT( v4l2-lib )

#my mozilla PC for openCV build
#set(OpenCV_DIR "/home/thomas/build/misc/opencv/build")
#my acer s3 ubuntu
set(OpenCV_DIR "/home/thomas/build/biotrump-cv/out/openCV/2.4.9.x-static")
#set(OpenCV_DIR "/home/thomas/build/biotrump-cv/out/openCV/2.4.9.x")
FIND_PACKAGE( OpenCV REQUIRED)

#openMP to avoid link error : omp_xxx error!
# undefined reference to `GOMP_loop_dynamic_start
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
#$ENV{VAR} to get shell env
set(V4L2_LIB_DIR $ENV{V4L2_LIB_DIR})
set(V4L2_LIB_OUT $ENV{V4L2_LIB_OUT} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${V4L2_LIB_OUT})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${V4L2_LIB_OUT})

message("V4L2_LIB_OUT is ${V4L2_LIB_OUT}")

#    set_target_properties( v4l2-lib
#    PROPERTIES
#    ARCHIVE_OUTPUT_DIRECTORY "${V4L2_LIB_OUT}"
#    LIBRARY_OUTPUT_DIRECTORY "${V4L2_LIB_OUT}"
#    RUNTIME_OUTPUT_DIRECTORY "${V4L2_LIB_OUT}"
#)

#build a static lib
ADD_LIBRARY(v4l2-lib STATIC
	Webcam.cpp
	CameraFrame.cpp
	capture-v4l2.c
	)

#dynamic or static link to dependent openCV libs
TARGET_LINK_LIBRARIES( v4l2-lib ${OpenCV_LIBS} )
