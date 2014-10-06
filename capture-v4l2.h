#ifndef _H_CAPTURE_V4L2_H
#define _H_CAPTURE_V4L2_H
/*
	<thomas tsai>thomas@biotrump.com
*/
#include <linux/videodev2.h>	//V4L2 definitions, /include/uapi/linux/videodev2.h

int GetAutoWhiteBalance(int fd);
int SetAutoWhiteBalance(int fd, int enable);
int getVideoFMT(int fd);
int setVideoFMT(int fd, __u32 pixelformat);

int GetManualExposure(int fd);
int SetManualExposure(int fd, int val);
int GetAutoExposure(int fd);
int SetAutoExposure(int fd, int type);
int GetAutoExposureAutoPriority(int fd);
int SetAutoExposureAutoPriority(int fd, int p);

//int SetManualExposureMode(int fd);
//int SetAutoExposureExt(int fd, int ap, int exp_auto, int metering);
//int SetManualExposureExt(int fd, int value);

#endif

