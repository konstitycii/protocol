#ifndef resulthread_h
#define resulthread_h

#include <asm/termbits.h>
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
#include <time.h>

#include <cstring>
#include <vector>
#include <fstream>
#include <iterator>
#include <ctime> 

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>


#define 	CLEAR(x) 		memset(&(x), 0, sizeof(x))
#define  	WIDTH   		1920
#define   	HEIGHT  		1080
#define   	FMT     		V4L2_PIX_FMT_RGBA32
#define     	dev_name		"/dev/video1"
#define 	dev_name_media		"/dev/v4l-subdev2"
#define	value_param		2
#define	tty_port_target	"/dev/ttymxc0"
#define	tty_port_pc		"/dev/ttyUSB0"
#define 	CRC_INIT 		0xFFFF 
#define 	BUF_SIZE 		10
#define 	POLYNOM 		0xA001  //x16 + x15 + x2 + 1 (reversed)
#define 	FILE_PATH 		1609264
#define	tty_speed		468000
#define STEP_UP 11


int k = 0;

struct buffer {
void   *start;
size_t length;
};

struct argument_photo {
	unsigned char* buffer;
	size_t length; 
};

/*typedef struct node {
	int 		vector;
	struct node 	*next;
} Node;

int push (Node **stack, unsigned char val) {
	Node *p = (Node *)malloc( sizeof( Node ) );
	int success = p != NULL;

	if ( success )
	{
		p->vector = val;
		p->next = *stack;
		*stack = p;
	}

	return success;
}

int pop (Node **stack, unsigned char *val) {
	int success = *stack != NULL;
	
	if(success){
		Node *point = *stack;
		*stack = (*stack) -> next;
		*val = point -> vector;
		free(point);
	}
}*/


const unsigned char auchCRCHi [256] =
{
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40
};

const unsigned char auchCRCLo [256] =
{
	0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
	0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
	0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
	0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
	0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
	0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
	0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
	0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
	0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
	0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
	0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
	0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
	0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
	0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
	0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};

void* photo (void*);
unsigned char preambula[1] = {0xFF};
unsigned char start_byte = 0xA5;
unsigned char to_PC_ARM [1] = {0x0A};
unsigned char module_vk = 0x0B;
unsigned char rezhim = 0x02;
unsigned int alarm_interval = 5000;
unsigned int photo_alarm_done = 0;
unsigned int photo_after_alarm_done = 0;

uint8_t SPHV = 0x08;

uint16_t calculation_crc (unsigned char buffer [1024], int length){
	int i = 1;
	uint8_t uIndex;
	uint8_t CRCh;
	uint8_t CRCl;
	
	CRCh = 0xFF;
	CRCl = 0xFF;
	for (i = 1; i < length; i++){
		uIndex = CRCh ^ buffer[i];
		//printf("buffer_crc[%d] = %x\n", i, buffer[i]);
		CRCh = CRCl ^ auchCRCHi [uIndex];
		CRCl = auchCRCLo [uIndex];
	}
	//printf("CRCh = %x\n", CRCh);
	//printf("CRCl = %x\n", CRCl);
	//return 0;
	return (CRCh << 8) | CRCl; // Возвращаем 16 битное значение CRC, для дальнейшей передачи необходимо разделить на две части 
	
}


static void xioctl(int fh, int request, void *arg){
	int r;

	do{
		r = ioctl(fh, request, arg);
	}while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (r == -1){
		fprintf(stderr, "error %d %s %d\n", errno, strerror(errno), request);
		exit(EXIT_FAILURE);
	}

}

//void* result (void*);

unsigned char *make_photo_timer(){
	using namespace 		cv;
	using namespace 		std;

	unsigned char*			buffer_photo = (unsigned char*)  malloc (sizeof(unsigned char)*WIDTH*HEIGHT);
	//argument_photo*		buffer_photo = (argument_photo*)argument;
	char*				val = (char*) malloc (sizeof(char)*4*WIDTH*HEIGHT);
	char*				BUFIMAGE = new char [WIDTH*HEIGHT];
	char*				BUFIMAGER = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEG = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEB = new char [WIDTH*HEIGHT];
	
	buffer*			buffers; 
	//buffer*			buffer_photo = (buffer *) argument;
	Mat				greyImg, imgR, imgG, imgB, imgM;
	
	int				fd = -1;
	int				i,j = 0;
	unsigned int			n_buffers;
	
	v4l2_format			fmt;
	v4l2_buffer			buf;
	v4l2_requestbuffers		req;
	v4l2_capability		device_params;
	v4l2_buf_type			type;
	
	//printf("start\n");
	
	fd = open(dev_name, O_RDWR | O_NONBLOCK);
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
	fmt.fmt.pix.pixelformat = FMT;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;
	xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (fmt.fmt.pix.pixelformat != FMT) {
		printf("Libv4l didn't accept ЭТОТ format. Can't proceed.\\n");
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
		BUFIMAGE [j] = 0.3*val [4*j] + 0.6*val[4*j + 1] + 0.1*val[4*j + 2];
		BUFIMAGER [j] = val [4*j];
		BUFIMAGEG [j] = val [4*j + 1];
		BUFIMAGEB [j] = val [4*j + 2];
	}
	
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_QBUF, &buf);
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);
	//printf("free\n");
	for (i = 0; i < n_buffers; ++i){
		munmap(buffers[i].start, buffers[i].length);
	}
	close(fd);
	free(val);
	greyImg = Mat(HEIGHT, WIDTH, CV_8U, BUFIMAGE);
	imgR = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGER);
	imgG = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGEG);
	imgB = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGEB);
	vector<Mat> channels;
	vector<int> compression_params;
	compression_params.push_back(IMWRITE_JPEG_QUALITY);
	compression_params.push_back(50);
	channels.push_back(imgB);
	channels.push_back(imgG);
	channels.push_back(imgR);
	merge(channels,imgM);
	//printf("delete\n");
	delete [] BUFIMAGE;
	delete [] BUFIMAGER;
	delete [] BUFIMAGEG;
	delete [] BUFIMAGEB;
	
	vector<int> param(2);
	char name[100];
	param[0] = IMWRITE_JPEG_QUALITY;
	param[1] = 70;//default(95) 0-100
	sprintf(name, "cropped%02d.jpg", k); 
	//k++;
	//printf("imwrite\n");
	//imwrite("starry.png", greyImg);
	//printf("imwrite\n");
	imwrite(name, imgM, compression_params);
	//vector<uchar> *buffer_img = (vector<uchar> *)argument;
	//printf("imencode\n");
	vector<uchar> buffer_img;
	imencode(".jpg", imgM, buffer_img, param);
	//printf("calloc\n"); 
	//buffer_photo = (argument_photo *) calloc (buffer_img.size(), sizeof(argument_photo));
	
	buffer_photo = &buffer_img[0];
	//printf("buffer = %x\n", buffer_photo[1]);
	//printf("buffer = %x\n", buffer_photo[buffer_img->size()-1]);
	//printf("k = %d\n", k);
	
	//buffer_photo -> length = buffer_img.size();
	//printf("lenght = %lx\n", buffer_photo -> length);
	//printf("Size buffer x16 = %lx\n", buffer_img->size());
	//buffer_photo -> start = buffer_img.begin();
	//memcpy(buffer_photo, buffer_img[0], buffer_img.size());
	//return buffer_photo;
	//free(buffer_photo);
	//buffer_img->clear();
	//printf("Size buffer x16 after clear = %lx\n", buffer_img->size());
	return buffer_photo;
	//return 0;
}

    long getEpochTimeShift(){
        struct timeval epochtime;
        struct timespec  vsTime;

        gettimeofday(&epochtime, NULL);
        clock_gettime(CLOCK_MONOTONIC, &vsTime);

        long uptime_ms = vsTime.tv_sec* 1000 + (long)  round( vsTime.tv_nsec/ 1000000.0);
        long epoch_ms =  epochtime.tv_sec * 1000  + (long) round( epochtime.tv_usec/1000.0);
        return epoch_ms - uptime_ms;
    }

    //stick this somewhere so that it runs once, on the startup of your capture process
    //  noting, if you hibernate a laptop, you might need to recalc this if you don't restart 
    //  the process after dehibernation
    long toEpochOffset_ms = getEpochTimeShift();


void *make_photo(void* argument){
	using namespace 		cv;
	using namespace 		std;

	unsigned char*			buffer_photo;// = (unsigned char*) argument;// = new unsigned char [WIDTH*HEIGHT];
	//argument_photo*		buffer_photo = (argument_photo*)argument;
	char*				val = (char*) malloc (sizeof(char)*4*WIDTH*HEIGHT);
	char*				BUFIMAGE = new char [WIDTH*HEIGHT];
	char*				BUFIMAGER = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEG = new char [WIDTH*HEIGHT];
	char*				BUFIMAGEB = new char [WIDTH*HEIGHT];
	
	buffer*			buffers; 
	//buffer*			buffer_photo = (buffer *) argument;
	Mat				greyImg, imgR, imgG, imgB, imgM;
	
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
	
	//printf("start\n");
	
	fd = open(dev_name, O_RDWR | O_NONBLOCK);
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
	fmt.fmt.pix.pixelformat = FMT;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;
	xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (fmt.fmt.pix.pixelformat != FMT) {
		printf("Libv4l didn't accept ЭТОТ format. Can't proceed.\\n");
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
	
		/*switch( buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK )
{
    case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
    {
        struct timespec uptime = {0};
        clock_gettime(CLOCK_MONOTONIC,&uptime);

        float const secs =
            (buf.timestamp.tv_sec - uptime.tv_sec) +
            (buf.timestamp.tv_usec - uptime.tv_nsec/1000.0f)/1000.0f;

        if( V4L2_BUF_FLAG_TSTAMP_SRC_SOE == (buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) )
            printf("%s: frame exposure started %.03f seconds ago\n",__FUNCTION__,-secs);
        else if( V4L2_BUF_FLAG_TSTAMP_SRC_EOF == (buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) )
            printf("%s: frame finished capturing %.03f seconds ago\n",__FUNCTION__,-secs);
        else printf("%s: unsupported timestamp in frame\n",__FUNCTION__);

        break;
    }

    case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
    case V4L2_BUF_FLAG_TIMESTAMP_COPY:
    default:
        printf("%s: no usable timestamp found in frame\n",__FUNCTION__);
}*/

	//long temp_ms = 1000 * buf.timestamp.tv_sec + (long) round(  buf.timestamp.tv_usec / 1000.0);
	//long epochTimeStamp_ms = temp_ms + toEpochOffset_ms ;

    //printf( "the frame's timestamp in epoch ms is: %ld\n", epochTimeStamp_ms);
	
	
	memcpy(val, buffers[buf.index].start, buf.bytesused);

	for (j = 0 ; j < (WIDTH*HEIGHT); j++){
		BUFIMAGE [j] = 0.3*val [4*j] + 0.6*val[4*j + 1] + 0.1*val[4*j + 2];
		BUFIMAGER [j] = val [4*j];
		BUFIMAGEG [j] = val [4*j + 1];
		BUFIMAGEB [j] = val [4*j + 2];
	}
	
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_QBUF, &buf);
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);
	//printf("free\n");
	for (i = 0; i < n_buffers; ++i){
		munmap(buffers[i].start, buffers[i].length);
	}
	close(fd);
	free(val);
	now = time(0);
	dt = ctime(&now);
	greyImg = Mat(HEIGHT, WIDTH, CV_8U, BUFIMAGE);
	imgR = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGER);
	imgG = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGEG);
	imgB = Mat(HEIGHT, WIDTH, CV_8UC1, BUFIMAGEB);
	vector<Mat> channels;
	vector<int> compression_params;
	compression_params.push_back(IMWRITE_JPEG_QUALITY);
	compression_params.push_back(50);
	channels.push_back(imgB);
	channels.push_back(imgG);
	channels.push_back(imgR);
	merge(channels,imgM);
	//printf("delete\n");
	delete [] BUFIMAGE;
	delete [] BUFIMAGER;
	delete [] BUFIMAGEG;
	delete [] BUFIMAGEB;
	
	vector<int> param(2);
	char name[100];
	param[0] = IMWRITE_JPEG_QUALITY;
	param[1] = 65;//default(95) 0-100
	sprintf(name, "cropped%02d.jpg", k); 
	//k++;
	//printf("imwrite\n");
	//imwrite("starry.png", greyImg);
	//printf("imwrite\n");
	imwrite(name, imgM, compression_params);
	putText(imgM,dt,Point(50,50),FONT_HERSHEY_DUPLEX,2,Scalar(255,255,255),2,false);
	vector<uchar> *buffer_img = (vector<uchar> *)argument;
	//printf("imencode\n");
	imencode(".jpg", imgM, *buffer_img, param);
	//printf("calloc\n"); 
	//buffer_photo = (argument_photo *) calloc (buffer_img.size(), sizeof(argument_photo));
	
	//buffer_photo = buffer_img;
	//printf("buffer = %x\n", buffer_photo[1]);
	//printf("buffer = %x\n", buffer_photo[buffer_img->size()-1]);
	//printf("k = %d\n", k);
	
	//buffer_photo -> length = buffer_img.size();
	//printf("lenght = %lx\n", buffer_photo -> length);
	//printf("Size buffer x16 = %lx\n", buffer_img->size());
	//buffer_photo -> start = buffer_img.begin();
	//memcpy(buffer_photo, buffer_img[0], buffer_img.size());
	//return buffer_photo;
	//free(buffer_photo);
	//buffer_img->clear();
	//printf("Size buffer x16 after clear = %lx\n", buffer_img->size());
	return 0;
}



int set_param_tty (int serial_port, struct termios2 tty){
	ioctl(serial_port, TCGETS2, &tty);
	
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag |= CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disassble interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	
	tty.c_cc[VMIN] = 0;
   	tty.c_cc[VTIME] = 0;
	
	tty.c_cflag &= ~CBAUD;
	tty.c_cflag |= CBAUDEX;
	
	tty.c_ispeed = tty_speed; // What a custom baud rate!
	tty.c_ospeed = tty_speed;
	
	ioctl(serial_port, TCSETS2, &tty);
	return 0;
}

unsigned short crc16(unsigned crc, char *buf, unsigned len);

/*unsigned char byte_staff_out(unsigned char* buffer = new unsigned char [2048], int lenght){
	int i;
	int j;
	unsigned char* output_buffer = new unsigned char [2048];
	i = 1;
	j = 1;
	output_buffer[0] = buffer[0];
	while(i < lenght){
		if(buffer[i] == 0xA5){
			output_buffer[j] = 0x1B;
			j++;
			output_buffer[j] = 0xC5;
		} else {
			if(buffer[i] == 0x1B){
				output_buffer[j] = 0x1B;
				j++;
				output_buffer[j] = 0x3B;
			} else {
				output_buffer[j] = buffer[i];
			}
		}
		i++;
		j++;
	}
	
	return output_buffer;
} */

#endif /* resulthread_h*/
