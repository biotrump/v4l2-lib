/*http://www.jayrambhia.com/blog/capture-v4l2/
 *https://gist.github.com/jayrambhia/5866483
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>	//V4L2 definitions, /include/uapi/linux/videodev2.h
//#include <uapi/linux/videodev2.h>	//V4L2 definitions, /include/uapi/linux/videodev2.h
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define		V4L2_IOCTL_RETRY	(20)

#define		__debug_log_	1

uint8_t *buffer;

//http://linuxtv.org/downloads/v4l-dvb-apis/extended-controls.html
static int xioctl(int fd, int request, void *arg)
{
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

int QueryCap(int fd, struct v4l2_capability *pcaps)
{
    struct v4l2_capability caps = {0};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, pcaps)){
        perror("Querying Capabilities");
        return 1;
    }
    return 0;
}

int QueryCropCap(int fd, struct v4l2_cropcap *pcropcap)
{
    pcropcap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (fd, VIDIOC_CROPCAP, pcropcap)) {
        perror("Querying Cropping Capabilities");
        return 1;
    }
}

int GetAutoWhiteBalance(int fd)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
	if (-1 == xioctl(fd, VIDIOC_G_CTRL, &ctrl)){
		perror("getting V4L2_CID_AUTO_WHITE_BALANCE");
	}
//	printf("V4L2_CID_AUTO_WHITE_BALANCE = 0x%x\n",ctrl.value );
	return ctrl.value;
}

int SetAutoWhiteBalance(int fd, int enable)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
	ctrl.value = enable;
	if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl)){
		perror("setting V4L2_CID_AUTO_WHITE_BALANCE");
	}
//	printf("V4L2_CID_AUTO_WHITE_BALANCE = (0x%x -> 0x%x)\n",enable, GetAutoWhiteBalance(fd) );
	return GetAutoWhiteBalance(fd) == enable;
}

/* v4l2-controls.h
enum  v4l2_exposure_auto_type {
	V4L2_EXPOSURE_AUTO = 0,
	V4L2_EXPOSURE_MANUAL = 1,
	V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
	V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
V4L2_CID_EXPOSURE_AUTO 	enum v4l2_exposure_auto_type
 	Enables automatic adjustments of the exposure time and/or iris aperture.
 	The effect of manual changes of the exposure time or iris aperture while
 	these features are enabled is undefined, drivers should ignore such requests.
 	Possible values are:
	V4L2_EXPOSURE_AUTO 	Automatic exposure time, automatic iris aperture.
	V4L2_EXPOSURE_MANUAL 	Manual exposure time, manual iris.
	V4L2_EXPOSURE_SHUTTER_PRIORITY 	Manual exposure time, auto iris.
	V4L2_EXPOSURE_APERTURE_PRIORITY 	Auto exposure time, manual iris.

V4L2_CID_EXPOSURE_ABSOLUTE 	integer
 	Determines the exposure time of the camera sensor. The exposure time is
 	limited by the frame interval. Drivers should interpret the values as 100 µs
 	units, where the value 1 stands for 1/10000th of a second, 10000 for
 	1 second and 100000 for 10 seconds.

V4L2_CID_EXPOSURE_AUTO_PRIORITY 	boolean
 	When V4L2_CID_EXPOSURE_AUTO is set to AUTO or APERTURE_PRIORITY, this
 	control determines if the device may dynamically vary the frame rate.
 	By default this feature is disabled (0) and the frame rate must remain constant.

V4L2_CID_EXPOSURE_BIAS 	integer menu
 	Determines the automatic exposure compensation, it is effective only
 	when V4L2_CID_EXPOSURE_AUTO control is set to AUTO, SHUTTER_PRIORITY or
 	APERTURE_PRIORITY. It is expressed in terms of EV, drivers should interpret
 	the values as 0.001 EV units, where the value 1000 stands for +1 EV.
	Increasing the exposure compensation value is equivalent to decreasing the
	exposure value (EV) and will increase the amount of light at the image
	sensor. The camera performs the exposure compensation by adjusting absolute
	exposure time and/or aperture.

V4L2_CID_EXPOSURE_METERING 	enum v4l2_exposure_metering
 	Determines how the camera measures the amount of light available for the
 	frame exposure. Possible values are:

	V4L2_EXPOSURE_METERING_AVERAGE 	Use the light information coming from the
		entire frame and average giving no weighting to any particular portion of
		the metered area.
	V4L2_EXPOSURE_METERING_CENTER_WEIGHTED 	Average the light information coming
		from the entire frame giving priority to the center of the metered area.
	V4L2_EXPOSURE_METERING_SPOT 	Measure only very small area at the center
		of the frame.
	V4L2_EXPOSURE_METERING_MATRIX 	A multi-zone metering. The light intensity
		is measured in several points of the frame and the the results are
		combined. The algorithm of the zones selection and their significance
		in calculating the final value is device dependent.
*/
static void autoExposureType(int type)
{
#if __debug_log_
   	switch(type){
   	case V4L2_EXPOSURE_AUTO:
   		printf("V4L2_EXPOSURE_AUTO[%d]\n",type);
   		break;
   	case V4L2_EXPOSURE_MANUAL:
   		printf("V4L2_EXPOSURE_MANUAL[%d]\n",type);
   		break;
   	case V4L2_EXPOSURE_SHUTTER_PRIORITY:
   		printf("V4L2_EXPOSURE_SHUTTER_PRIORITY[%d]\n",type);
   		break;
   	case V4L2_EXPOSURE_APERTURE_PRIORITY:
   		printf("V4L2_EXPOSURE_APERTURE_PRIORITY[%d]\n",type);
   		break;
   	default:
   		perror("unknown exposure type");
   	}
#endif
}

/*
V4L2_EXPOSURE_MANUAL and V4L2_EXPOSURE_APERTURE_PRIORITY are commonly used.
*/
int SetAutoExposure(int fd, int type)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_EXPOSURE_AUTO;
   	ctrl.value = type;
   	printf("SetAutoExposure=");
   	autoExposureType(type);
   	if (-1 == xioctl(fd,VIDIOC_S_CTRL,&ctrl)) {
      perror("setting V4L2_CID_EXPOSURE_AUTO");
      return -1;
	}
	return type;
}

int GetAutoExposure(int fd)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_EXPOSURE_AUTO;
	if (-1 == xioctl(fd, VIDIOC_G_CTRL, &ctrl)){
		perror("getting V4L2_CID_EXPOSURE_AUTO");
		return -1;
	}
	printf("GetAutoExposure=");
	autoExposureType(ctrl.value);
	return ctrl.value;
}

int SetAutoExposureAutoPriority(int fd, int p)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY ;
   	ctrl.value = p;
   	if (-1 == xioctl(fd,VIDIOC_S_CTRL,&ctrl)) {
      perror("setting V4L2_CID_EXPOSURE_AUTO_PRIORITY");
      return -1;
	}
	return p;
}

int GetAutoExposureAutoPriority(int fd)
{
	struct v4l2_control ctrl ={0};
	ctrl.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY ;
   	if (-1 == xioctl(fd,VIDIOC_G_CTRL,&ctrl)) {
      perror("getting V4L2_CID_EXPOSURE_AUTO_PRIORITY");
      return -1;
	}
	return ctrl.value;
}

/*
	The exposure time is limited by the frame interval. Drivers should interpret
	the values as 100 µs units, where the value 1 stands for 1/10000th of a second,
	10000 for 1 second and 100000 for 10 seconds.
	if 30fps is the goal, each frame takes 1/30s = 33ms = 33*10*100us =
	330 * 100us. It depends on the ambient light to tuning exposure time.
*/
int SetManualExposure(int fd, int val)
{
	struct v4l2_control ctrl ={0};
	if(SetAutoExposure(fd, V4L2_EXPOSURE_MANUAL) != V4L2_EXPOSURE_MANUAL){
		perror("setting V4L2_EXPOSURE_MANUAL");
		return -1;
	}

	ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
   	ctrl.value = val;
   	if (-1 == xioctl(fd,VIDIOC_S_CTRL,&ctrl)) {
      perror("setting V4L2_CID_EXPOSURE_ABSOLUTE");
      return -1;
	}
	return val;
}

int GetManualExposure(int fd)
{
	struct v4l2_control ctrl ={0};
/*	if(GetAutoExposure(fd) != V4L2_EXPOSURE_MANUAL){
		perror("SetManualExposure is not in V4L2_EXPOSURE_MANUAL");
		return -1;
	}
*/
	ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
   	if (-1 == xioctl(fd,VIDIOC_G_CTRL,&ctrl)) {
      perror("getting V4L2_EXPOSURE_MANUAL");
      return -1;
	}
	printf("GetManualExposure=%d\n",ctrl.value);
	return ctrl.value;
}

#if 0
/*
If the fps must be constant, V4L2_CID_EXPOSURE_AUTO_PRIORITY should be disabled.
V4L2_CID_EXPOSURE_METERING should be V4L2_EXPOSURE_METERING_CENTER_WEIGHTED
*/
int SetManualExposureMode(int fd)
{
	struct v4l2_ext_control controlList[2];
	struct v4l2_ext_controls controls;
	int i=0;
	//configure exposure priority
	controlList[0].id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;

	//configure exposure mode
	controlList[1].id = V4L2_CID_EXPOSURE_AUTO;

	controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
	controls.count = 2;
	controls.controls = controlList;

	for(i=0; i< V4L2_IOCTL_RETRY; i++)
	{
		controlList[0].value = 0; // disabled
		controlList[1].value = 1; // manual mode

	    if (-1 == xioctl (fd, VIDIOC_S_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error setting exposure mode!");
	    	usleep( 5000 );
	    	continue;
	    }

	    //test if value is set correctly
	    if (-1 == xioctl (fd, VIDIOC_G_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error testing exposure mode!");
	    	usleep( 5000 );
	    	continue;
	    }

	    if(controls.controls[0].value == 0 && controls.controls[1].value == 1)
	    	return 1;
	    else
	    {
	    	perror("[V4LCamera] Extended control value not set yet, trying again in 5ms!");
	    	usleep( 5000 );
	    }
	}

	perror("[V4LCamera] Could not set extended control value for 20 times, giving up!");
    return 0;
}

int SetAutoExposureExt(int fd, int ap, int exp_auto, int metering)
{
	struct v4l2_ext_control controlList[3];
	struct v4l2_ext_controls controls;
	int i=0;
	printf("[V4LCamera] Setting exposure mode!\n");
	//configure exposure priority
	controlList[0].id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;

	//configure exposure mode
	controlList[1].id = V4L2_CID_EXPOSURE_AUTO;

	//configure exposure metering
//	controlList[2].id = V4L2_EXPOSURE_METERING_CENTER_WEIGHTED;//V4L2_CID_EXPOSURE_METERING;

	controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
//	controls.count = 3;
	controls.count = 2;
	controls.controls = controlList;

	for(i=0; i< V4L2_IOCTL_RETRY; i++)//try counts
	{
		controlList[0].value = ap; 			// disabled auto priority, so the frame rate can be kept!
		controlList[1].value = exp_auto; 		// auto mode
//		controlList[2].value = metering; 	// exposure metering
	    if (-1 == xioctl (fd, VIDIOC_S_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error setting exposure mode!\n");
	    	usleep( 5000 );
	    	continue;
	    }

	    //test if value is set correctly
	    if (-1 == xioctl (fd, VIDIOC_G_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error testing exposure mode!\n");
	    	usleep( 5000 );
	    	continue;
	    }

	    if(controls.controls[0].value == 0 && controls.controls[1].value == 0)
	    	return 1;
	    else
	    {
	    	perror("[V4LCamera] Extended control value not set yet, trying again in 5ms!");
	    	usleep( 5000 );
	    }
	}

	perror("[V4LCamera] Could not set extended control value for 20 times, giving up!");
    return 0;
}

int SetManualExposureExt(int fd, int value)
{
	struct v4l2_ext_control controlList[1];
	struct v4l2_ext_controls controls;
	int i=0;
	perror("[V4LCamera] Setting exposure!");
	SetManualExposureMode(fd);

	//configure exposure priority
	controlList[0].id = V4L2_CID_EXPOSURE_ABSOLUTE;
	controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
	controls.count = 1;
	controls.controls = controlList;

	for(i=0; i< V4L2_IOCTL_RETRY; i++)
	{
		controlList[0].value = value;

	    if (-1 == xioctl (fd, VIDIOC_S_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error setting exposure mode!");
	    	usleep( 5000 );
	    	continue;
	    }

	    //test if value is set correctly
	    if (-1 == xioctl (fd, VIDIOC_G_EXT_CTRLS, &controls))
	    {
	    	perror("[V4LCamera] Error testing exposure mode!");
	    	usleep( 5000 );
	    	continue;
	    }

	    if(controls.controls[0].value == value)
	    	return 1;
	    else
	    {
	    	perror("[V4LCamera] Extended control value not set yet, trying again in 5ms!");
	    	usleep( 5000 );
	    }
	}

	perror("[V4LCamera] Could not set extended control value for 20 times, giving up!");
    return 0;
}
#endif

int EnumVideoFMT(int fd)
{
	int support_grbg10 = 0;
	struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("\n  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: [%c][%c], [%s]\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }
}

int GetVideoFMT(int fd, struct v4l2_format *pfmt)
{
    int i;
//    struct v4l2_format fmt = {0};
    char fourcc[5] = {0};
    pfmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	for(i = 0; i < 20; i ++){
		if (-1 == xioctl(fd, VIDIOC_G_FMT, pfmt))
		{
		    perror("getting Pixel Format");
			usleep(5000);
			continue;
		}else{
		strncpy(fourcc, (char *)&pfmt->fmt.pix.pixelformat, 4);
		printf( "Gotten Camera Mode:\n"
		    "  Width: %d\n"
		    "  Height: %d\n"
		    "  PixFmt: %s\n"
		    "  Field: %d\n"
		    "  priv: 0x%x\n",
		    pfmt->fmt.pix.width,
		    pfmt->fmt.pix.height,
		    fourcc,
		    pfmt->fmt.pix.field,
		    pfmt->fmt.pix.priv);
		    break;
		}
	}
}

int SetVideoFMT(int fd, struct v4l2_format fmt)
{
    int i;
//    struct v4l2_format fmt = {0};
    char fourcc[5] = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//    fmt.fmt.pix.width = 640;
//    fmt.fmt.pix.height = 480;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
//    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
//    fmt.fmt.pix.pixelformat = pixelformat;//V4L2_PIX_FMT_YUYV;
//    fmt.fmt.pix.field = V4L2_FIELD_NONE;
//	for(i = 0; i < 20; i ++){
		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
		{
		    perror("Setting Pixel Format");
			//usleep(10000);
			//continue;
		}else{
		strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
		printf( "Selected Camera Mode:\n"
		    "  Width: %d\n"
		    "  Height: %d\n"
		    "  PixFmt: %s\n"
		    "  Field: %d\n",
		    fmt.fmt.pix.width,
		    fmt.fmt.pix.height,
		    fourcc,
		    fmt.fmt.pix.field);
		    //break;
		}
	//}
	return 0;
}

/*
void Webcam::GetParam(v4l2_streamparm &param) {
	param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(m_fd, VIDIOC_G_PARM, &param);

	if (ret == -1) {
		throw std::runtime_error("unable to fetch device parameters");
	}
}

void Webcam::SetParam(v4l2_streamparm &param) {
	int ret = ioctl(m_fd, VIDIOC_S_PARM, &param);

	if (ret == -1) {
		throw std::runtime_error("unable to change device parameters");
	}
}

double Webcam::GetFPS(void) const {
	// TODO: Verify that denom is not zero.
	double numer = m_param.parm.capture.timeperframe.numerator;
	double denom = m_param.parm.capture.timeperframe.denominator;
	return denom / numer;
}

void Webcam::SetFPS(uint32_t fps) {
	m_param.parm.capture.timeperframe.numerator   = 1;
	m_param.parm.capture.timeperframe.denominator = fps;
	SetParam(m_param);

	uint32_t fps_new = m_param.parm.capture.timeperframe.denominator
	                 / m_param.parm.capture.timeperframe.numerator;
	if (fps != fps_new) {
		throw std::invalid_argument("unsupported frame rate");
	}
}
*/

void SetFPSParam(int fd, uint32_t fps) 
{
	struct v4l2_streamparm param;
    memset(&param, 0, sizeof(param));
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    param.parm.capture.timeperframe.numerator = 1;
    param.parm.capture.timeperframe.denominator = fps;  
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &param)){
		perror("unable to change device parameters");
		return ;
	}

	if(param.parm.capture.timeperframe.numerator){
		double fps_new = param.parm.capture.timeperframe.denominator
	                 / param.parm.capture.timeperframe.numerator;
		if ((double)fps != fps_new) {
			printf("unsupported frame rate [%d,%f]\n", fps, fps_new);
			return;
		}else{
			printf("new fps:%u , %u/%u\n",fps, param.parm.capture.timeperframe.denominator,
			param.parm.capture.timeperframe.numerator);
		}
	}
}

uint32_t GetFPSParam(int fd, double fps, struct v4l2_frmivalenum *pfrmival)
{
    struct v4l2_frmivalenum frmival[10];
    float fpss[10];
    int i=0;
/*
    memset(&frmival,0,sizeof(frmival));
    frmival.pixel_format = fmt;
    frmival.width = width;
    frmival.height = height;*/
    memset(fpss,0,sizeof(fpss));
    while(pfrmival->index < 10){
	    if (-1 == xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, pfrmival)){
		    perror("getting VIDIOC_ENUM_FRAMEINTERVALS");
		    break;
		}
		frmival[pfrmival->index] = *pfrmival;
        if (pfrmival->type == V4L2_FRMIVAL_TYPE_DISCRETE){
	    	double f;
        	f = (double)pfrmival->discrete.denominator/pfrmival->discrete.numerator;
        	printf("[%u/%u]\n", pfrmival->discrete.denominator, 
        						pfrmival->discrete.numerator);
            printf("[%dx%d] %f fps\n", pfrmival->width, pfrmival->height,f);
            
			fpss[pfrmival->index]=f;
			frmival[pfrmival->index]=*pfrmival;
        }else{
        	double f1,f2;
        	f1 = (double)pfrmival->stepwise.max.denominator/pfrmival->stepwise.max.numerator;
        	f2 = (double)pfrmival->stepwise.min.denominator/pfrmival->stepwise.min.numerator;
            printf("[%dx%d] [%f,%f] fps\n", pfrmival->width, pfrmival->height,f1,f2);
       	}
       	printf("idx=%d\n", pfrmival->index);
       	pfrmival->index++;
    }
    /* list is in increasing order */
    if(pfrmival->index){
    	i = pfrmival->index;
	    while(--i >= 0){
    		if(fps <= fpss[i] ){
    			break;
    		}
    	}
    	*pfrmival = frmival[i];
    	printf("found[%f,%f]\n", fps, fpss[i]);
    }
    return (uint32_t)fpss[i];
}

int PrintFrameInterval(int fd, unsigned int fmt, unsigned int width, unsigned int height)
{
    struct v4l2_frmivalenum frmival;
    memset(&frmival,0,sizeof(frmival));
    frmival.pixel_format = fmt;
    frmival.width = width;
    frmival.height = height;
    while(1){
	    if (-1 == xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival)){
		    perror("getting VIDIOC_ENUM_FRAMEINTERVALS");
		    return -1;
		}

        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE){
           	printf("[%u/%u]\n", frmival.discrete.denominator, 
        						frmival.discrete.numerator);
            printf("[%dx%d] %f fps\n", width, height,
            1.0*frmival.discrete.denominator/frmival.discrete.numerator);
		}else
            printf("[%dx%d] [%f,%f] fps\n", width, height,
            1.0*frmival.stepwise.max.denominator/frmival.stepwise.max.numerator,
            1.0*frmival.stepwise.min.denominator/frmival.stepwise.min.numerator);
        frmival.index++;
    }
    return 0;
}

int EnumFrameRate(int fd, unsigned int format)
{
    unsigned int width=0, height=0;;
    struct v4l2_frmsizeenum frmsize;
    memset(&frmsize,0,sizeof(frmsize));
    frmsize.pixel_format = format; //V4L2_PIX_FMT_JPEG;
	printf("\nEnumFrameRate:\n");
    while(1){
	    if (-1 == xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)){
		    perror("getting VIDIOC_ENUM_FRAMESIZES");
		    return -1;
		}
		printf("frmsize.type=%d\n", frmsize.type);
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE){
            PrintFrameInterval(fd, frmsize.pixel_format, frmsize.discrete.width, 
            frmsize.discrete.height);
        }else{
            for (width=frmsize.stepwise.min_width; width< frmsize.stepwise.max_width; 
            	width+=frmsize.stepwise.step_width)
                for (height=frmsize.stepwise.min_height; 
                	height< frmsize.stepwise.max_height; 
                	height+=frmsize.stepwise.step_height)
                    PrintFrameInterval(fd, frmsize.pixel_format, width, height);
        }
        frmsize.index++;
    }
    return 0;
}

int print_caps(int fd)
{
    struct v4l2_capability caps = {0};
    struct v4l2_cropcap cropcap = {0};

    if (!QueryCap(fd, &caps)){
	    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,                caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);
	}

    if (!QueryCropCap(fd, &cropcap)){
    	printf( "Camera Cropping:\n"
        "  Bounds: %dx%d+%d+%d\n"
        "  Default: %dx%d+%d+%d\n"
        "  Aspect: %d/%d\n",
        cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
        cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
        cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
	}
    EnumVideoFMT(fd);
    //int support_grbg10 = 0;
    /*
    if (!support_grbg10)
    {
        printf("Doesn't support GRBG10.\n");
        return 1;
    }*/

	GetAutoWhiteBalance(fd);

    return 0;
}

int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }

    buffer = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);

    return 0;
}

int capture_image(int fd, char* windowname)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return 1;
    }

    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return 1;
    }

    if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return 1;
    }
    printf ("show image\n");

    IplImage* frame;
    CvMat cvmat = cvMat(480, 640, CV_8UC3, (void*)buffer);
    frame = cvDecodeImage(&cvmat, 1);
//    cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
    cvShowImage(windowname, frame);
//    cvWaitKey(0);
//    cvSaveImage("image.jpg", frame, 0);

    return 0;
}


#if 0
int main()
{
        int fd;

        fd = open("/dev/video0", O_RDWR);
        if (fd == -1)
        {
                perror("Opening video device");
                return 1;
        }
        if(print_caps(fd))
            return 1;

        if(init_mmap(fd))
            return 1;
        int i;
        for(i=0; i<5; i++)
        {
            if(capture_image(fd))
                return 1;
        }
        close(fd);
        return 0;
}
#endif

