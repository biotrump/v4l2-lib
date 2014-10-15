cmake_minimum_required(VERSION 2.8)
PROJECT( v4l2-lib )

#There are three kinds of target files that may be built: archive, library, and runtime.
#Executables are always treated as runtime targets.
#Static libraries are always treated as archive targets.
#Module libraries are always treated as library targets.
#For non-DLL platforms shared libraries are treated as library targets.
#For DLL platforms the DLL part of a shared library is treated as a runtime target and
#the corresponding import library is treated as an archive target.
#All Windows-based systems including Cygwin are DLL platforms.
#This property is initialized by the value of the variable CMAKE_RUNTIME_OUTPUT_DIRECTORY
#if it is set when a target is created.
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${V4L2_LIB_OUT})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${V4L2_LIB_OUT})
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${V4L2_LIB_OUT})

#build a static lib
ADD_LIBRARY(v4l2-lib STATIC
	Webcam.cpp
	CameraFrame.cpp
	capture-v4l2.c
	)

#dynamic or static link to dependent openCV libs
TARGET_LINK_LIBRARIES( v4l2-lib ${OpenCV_LIBS} )