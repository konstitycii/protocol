#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <math.h>
#include <typeinfo>
#include <iostream>
#include <pthread.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>

#include "resulthread.h"

#define   	FMT_video0    		V4L2_PIX_FMT_SGRBG8
#define     	dev_name_0		"/dev/video0"

/*static void xioctl(int fh, int request, void *arg){
	int r;

	do{
		r = ioctl(fh, request, arg);
	}while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (r == -1){
		fprintf(stderr, "error %d %s %d\n", errno, strerror(errno), request);
		exit(EXIT_FAILURE);
	}

}
*/

void *make_photo_video0(void* argument){
	using namespace 		cv;
	using namespace 		std;

	unsigned char*			buffer_photo;
	vector<uchar>*			buffer_img = (vector<uchar> *)argument;
	char*				val = (char*) malloc (sizeof(char)*WIDTH*HEIGHT);
	char*				BUFIMAGE = new char [WIDTH*HEIGHT];
	char*				BUFIMAGER = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEG = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEB = new char [WIDTH*HEIGHT];
	
	buffer*			buffers; 
	Mat				greyImg, imgR, imgG, imgB, imgM, colorImg;
	
	int				fd = -1;
	int				i,j = 0;
	unsigned int			n_buffers;
	
	v4l2_format			fmt;
	v4l2_buffer			buf;
	v4l2_requestbuffers		req;
	v4l2_capability		device_params;
	v4l2_buf_type			type;
	
	time_t 			now;
	char* 				dt;
	
	
	fd = open(dev_name_0, O_RDWR | O_NONBLOCK);
	if (fd < 0){
		perror("Cannot open device video1");
		exit (EXIT_FAILURE);
	}
	
	CLEAR(device_params);
	xioctl(fd,VIDIOC_QUERYCAP, &device_params);
	
	CLEAR(fmt);
	fmt.type 		 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = WIDTH;
	fmt.fmt.pix.height      = HEIGHT;
	fmt.fmt.pix.pixelformat = FMT_video0;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;
	xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (fmt.fmt.pix.pixelformat != FMT_video0) {
		printf("Libv4l didn't accept ЭТОТ format. Can't proceed video0.\\n");
		exit(EXIT_FAILURE);
	}
	
	CLEAR(req);
	req.count = 1;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_REQBUFS, &req);
	
	buffers = (buffer*) calloc (req.count, sizeof(*buffers));
	if(!buffers){
		perror("Out of memory\n");
		exit(EXIT_FAILURE);
	}
	
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		xioctl(fd, VIDIOC_QUERYBUF, &buf);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL, buf.length,
			  PROT_READ | PROT_WRITE, MAP_SHARED,
			  fd, buf.m.offset);
		if (MAP_FAILED == buffers[n_buffers].start) {
		    printf("MAP fail. Can't proceed.\\n");perror("mmap");
		    printf("MAP fail. Can't proceed.\\n");
		    exit(EXIT_FAILURE);
		}
	}
	
	for (i = 0; i < n_buffers ; ++i) {
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		xioctl(fd, VIDIOC_QBUF, &buf);
	}
	
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMON, &type);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_DQBUF, &buf);
	
	memcpy(val, buffers[buf.index].start, buf.bytesused);

	for (j = 0 ; j < (WIDTH*HEIGHT); j++){
		BUFIMAGE [j] = val [j];
	}
	
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_QBUF, &buf);
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);
	
	for (i = 0; i < n_buffers; ++i){
		munmap(buffers[i].start, buffers[i].length);
	}
	close(fd);
	
	now = time(0);
	dt = ctime(&now);
	greyImg = Mat(HEIGHT, WIDTH, CV_8U, BUFIMAGE);
	
	
	vector<int> param(2);
	char name[100];
	param[0] = IMWRITE_JPEG_QUALITY;
	param[1] = 65;//default(95) 0-100
	sprintf(name, "cropped%02d.jpg", k); 
	
	imwrite(name, greyImg);
	cvtColor(greyImg, colorImg, COLOR_BayerGR2RGB);
	imwrite("cropped.jpg", colorImg);
	putText(colorImg,dt,Point(50,50),FONT_HERSHEY_DUPLEX,2,Scalar(255,120,255),2,false);
	
	
	imencode(".jpg", colorImg, *buffer_img, param);
	
	return 0;
}
