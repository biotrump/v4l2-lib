cmake_minimum_required(VERSION 2.8)
#C and C++ APIs for V4L2 under openCV
PROJECT( v4l2-lib )

#my mozilla PC for openCV build
#set(OpenCV_DIR "/home/thomas/build/misc/opencv/build")
#my acer s3 ubuntu
set(OpenCV_DIR "/home/thomas/build/biotrump-cv/out/openCV/2.4.9.x-static")
FIND_PACKAGE( OpenCV REQUIRED)

#openMP to avoid link error : omp_xxx error!
# undefined reference to `GOMP_loop_dynamic_start
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#build a static lib
ADD_LIBRARY(v4l2-lib STATIC
	Webcam.cpp
	CameraFrame.cpp
	capture-v4l2.c 
	)

#dynamic or static link to dependent openCV libs
TARGET_LINK_LIBRARIES( v4l2-lib ${OpenCV_LIBS} )
