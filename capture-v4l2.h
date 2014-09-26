#ifndef _H_CAPTURE_V4L2_H
#define _H_CAPTURE_V4L2_H
/*
	<thomas tsai>thomas@biotrump.com
*/

int GetAutoWhiteBalance(int fd);
int SetAutoWhiteBalance(int fd, int enable);
int getVideoFMT(int fd);
int setVideoFMT(int fd);

#endif

