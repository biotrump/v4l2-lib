#ifndef WEBCAM_HPP_
#define WEBCAM_HPP_

#include <errno.h>
#include <list>
#include <string>
#include <vector>
#include <stdint.h>
#include <sys/time.h>
#include <linux/videodev2.h>

#include "CameraFrame.hpp"

class Webcam {
public:
	struct Resolution {
		uint32_t width;
		uint32_t height;
	};

	struct Buffer {
		size_t       len;
		void        *ptr;
		v4l2_buffer *buf;
	};
	Webcam(int fd, size_t nbufs);
	Webcam(std::string file, size_t nbufs);
	Webcam(int fd);
	~Webcam(void);

	bool IsStreaming(void) const;
	void SetStreaming(bool streaming);

	double GetFPS(void) const;
	void SetFPS(uint32_t fps);

	uint32_t GetWidth(void) const;
	uint32_t GetHeight(void) const;
	void SetResolution(uint32_t width, uint32_t height);

	void SetControl(int type, int value);
	int GetControl(int type);

	CameraFrame &GetFrame(CameraFrame &frame);

	std::list<std::string> GetControls(void) const;
	std::list<uint32_t>    GetPixelFormats(void) const;
	std::list<Resolution>  GetResolutions(uint32_t pixel_format) const;
	std::list<double>      GetFPSs(uint32_t pixel_format, Resolution res) const;

private:
	int    m_fd;
	bool   m_streaming;
	size_t m_nbufs;
	v4l2_streamparm     m_param;
	v4l2_format         m_fmt_pix;
	std::vector<Buffer> m_bufs;

	Webcam(Webcam const &src);
	Webcam &operator=(Webcam const &src);

	void GetParam(v4l2_streamparm &param);
	void SetParam(v4l2_streamparm &param);

	void GetFormat(v4l2_format &fmt);
	void SetFormat(v4l2_format &fmt);

	void AllocateBuffer(size_t index);
	void DeallocateBuffer(size_t index);
};

#endif
