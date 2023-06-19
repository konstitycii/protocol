#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <deque>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
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
#include <ctime>
#include <chrono>
#include <atomic>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring> // memset()

#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include "resulthread.h"
#include "photo_video1.h"


#define PORT 14100
#define BUFFER_SIZE 1024
extern "C" {
    #include <linux/i2c-dev.h>
    #include "i2c/smbus.h"
}

#define CLEAR(x) memset(&(x), 0, sizeof(x))//очистка
#define   WIDTH   1920                       // The width of the picture
#define   HEIGHT  1080                       // The height of the picture
#define   FMT     V4L2_PIX_FMT_SGRBG8 // формат поддерживаемый матрицей (V4L2_PIX_FMT_SGRBG8 работает, но белый экран)
#define mydelay usleep (1000)// минимальная возможная задержка шага 100 мкс
#define pwmmydelay usleep (1000) // задержка между шагами
#define I2C4_PATH "/dev/i2c-3"
#define I2C2_PATH "/dev/i2c-1"
const unsigned default_port = 3425; // порт, по которому будет стучаться клиент
const int number_of_seconds = 5;   // кол-во секунд, что сервер будет хранить в памяти
#define INPUT_ADDR 0x22
#define INPUT_REG_P0 0x04
#define dev_name "/dev/video0"
const double default_fps = 25;      // должно совпадать с таким же значением из client.cpp
cv::VideoWriter writer;
using namespace cv;
using namespace std;
int grows = 1920; // высота
int gcols = 1080; // и ширина текущего видео захвата
deque<Mat> q;  // очередь с кадрами
mutex qmutex;  // мьютекс для охраны очереди
int working = 0;

/*
//структура для получения кадра
       struct buffer {
               void   *start;
               int64_t length;

       };
*/
/*char*                  	BUFIMAGEA = new char [WIDTH*HEIGHT];
       char*                    	BUFIMAGER = new char [WIDTH*HEIGHT];
       char*                    	BUFIMAGEG = new char [WIDTH*HEIGHT];
       char*                    	BUFIMAGEB = new char [WIDTH*HEIGHT];
       char*                    	BUFIMAGEY = new char [WIDTH*HEIGHT];*/


       v4l2_format			fmt; // формат видео
       v4l2_buffer              	buf; //
       v4l2_requestbuffers      	req; //
       v4l2_capability          	device_params; //параметры видеомодуля
       v4l2_buf_type            	type;
       fd_set                   	fds;
       timeval                  	tv;

       int				r, fd = -1;

       unsigned int			i, n_buffers;

       buffer*			buffers;

       char*				val = (char*)malloc(sizeof(char)*1*WIDTH*HEIGHT);

       using namespace cv;
       using namespace std;


       unsigned char read_buffer[1024];
       int				num_bytes;
       int  bytes_read = 0;
       int o = 0;
       int 				read_operation = 16;
       unsigned char get_buffer[256];
       long int				count_bytes = 0;
       unsigned char number_data;
       unsigned char byte_bytestuff = 0x00;
   	unsigned char buffer_bytestuff[1] = {0x00};
	uint16_t			CRC16;
	uint16_t			CRC16_get;
	unsigned char translate_buffer[2048] = {0};
	unsigned char output_buffer[2048] = {0};
	unsigned char number_stop_kadr;
	int number_out_data;
	unsigned char H_adress_begin_stop_kadr;
		unsigned char L_adress_begin_stop_kadr;
		unsigned char M_adress_begin_stop_kadr;
		unsigned char H_size_stop_kadr;
		unsigned char M_size_stop_kadr;
		unsigned char L_size_stop_kadr;
		time_t 			new_time = 0;
		unsigned char val_from;
		struct tm   get_time = {0};

		int				step_preambula;
		unsigned int			j, start_time, end_time, search_time;
		int test_byte = 0;
		int size_kadr;
		int start_kadr;
		/*unsigned char start_byte = 0xA5;
		unsigned char module_vk = 0x0B;
		unsigned char rezhim = 0x02;*/
		timeval 			time_field;
		int				result_time;
		vector<uchar> photo_alarm;
		int make_photo_after_alarm = 0;
		//int photo_after_alarm_done = 0;
		int make_photo_stop = 0;
		vector<uchar> photo_0_before_alarm;
		vector<uchar> photo_1_before_alarm;
		vector<uchar> photo_2_before_alarm;
		vector<uchar> photo_after_alarm;
		int step_up = 0;
		//unsigned char preambula[1] = {0xFF};
		int status_thread;
		pthread_t photo_thread;
		/*const unsigned char auchCRCHi [256] =
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

*/


       /*static void xioctl(int fh, int request, void *arg)//настройка параметров камеры
       {
               int r;

               do {
                       r = ioctl(fh, request, arg);
               } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

               if (r == -1) {
                       fprintf(stderr, "error %d, %s\\n", errno, strerror(errno), request);
                       exit(EXIT_FAILURE);
               }
       }
*/


//блок Таймера
       class Timer {
       	std::atomic<bool> active{true};

           public:
               void setTimeout(auto function, int delay);
               void setInterval(auto function, int interval);
               void stop();
               //void start();

       };

       void Timer::setTimeout(auto function, int delay) {
           active = true;
           std::thread t([=]() {
               if(!active.load()) return;
               std::this_thread::sleep_for(std::chrono::milliseconds(delay));
               if(!active.load()) return;
               function();
           });
           t.detach();
       }

       void Timer::setInterval(auto function, int interval) {
           active = true;
           std::thread t([=]() {
               while(active.load()) {
                   std::this_thread::sleep_for(std::chrono::milliseconds(interval));
                   if(!active.load()) return;
                   function();
               }
           });
           t.detach();
       }

       void Timer::stop() {
           active = false;
       }



       // структура для получения значения из потока с плавающей запятой
       typedef struct someArgs_tag {
           int id;
           const char *msg;
           double out[2];
       } someArgs_t;




/*
       //блок подсчета CRC
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
*/
   	Timer timer_photo;




       // блок, в котором идет накопление кадров в буфер длиною 5 секунд
             void video_grabber_thread() {
                 cout << "[VIDEO_GRABBER] : start" << endl;

                 // очищаем очередь перед началом работы
                 {
                     lock_guard<mutex> lk(qmutex);
                     q.clear();
                 }

                 printf("BEGIN\n");

                         fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);//открытие камеры

                         if (fd < 0) {
                                 perror("Cannot open device");
                                 exit(EXIT_FAILURE);
                         }

                         CLEAR(device_params);

                         if (ioctl(fd, VIDIOC_QUERYCAP, &device_params) == -1)// получение параметров видеомодуля из драйвера
                         {
                           printf ("\"VIDIOC_QUERYCAP\" error %d, %s\n", errno, strerror(errno));
                           exit(EXIT_FAILURE);
                         }

                         CLEAR(fmt);

                 //настройка формата
                         fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                         fmt.fmt.pix.width       = WIDTH;
                         fmt.fmt.pix.height      = HEIGHT;
                         fmt.fmt.pix.pixelformat = FMT;
                         fmt.fmt.pix.field       = V4L2_FIELD_NONE;
                         xioctl(fd, VIDIOC_S_FMT, &fmt);
                         if (fmt.fmt.pix.pixelformat != FMT) {
                                 printf("Libv4l didn't accept ЭТОТ format. Can't proceed.\\n");
                                 exit(EXIT_FAILURE);
                         }

                         CLEAR(req);//очистка структуры запросов

                         req.count = 4;//количество буферов
                         req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                         req.memory = V4L2_MEMORY_MMAP;
                         xioctl(fd, VIDIOC_REQBUFS, &req);

                         buffers =(buffer*)calloc(req.count, sizeof(*buffers));//выделение памяти под буферы
                         printf("buffers\n");
                         for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {//подготовка буферов к захвату
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

                 	printf("cycle\n");
                         for (i = 0; i < n_buffers ; ++i) {// поставка буферов в очередь на захват кадров
                                 CLEAR(buf);
                 		printf("qbuf\n");
                                 buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                                 buf.memory = V4L2_MEMORY_MMAP;
                                 buf.index = i;
                                 xioctl(fd, VIDIOC_QBUF, &buf);
                         }

                 	printf("on\n");
                         type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                         printf("V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
                         ioctl(fd, VIDIOC_STREAMON, &type);
      cout<<"streamon on "<<endl;

                         printf("endon\n");

                 i=0;


                 	VideoWriter writering("outpu517.avi", VideoWriter::fourcc('M','J','P','G'), 25, Size(WIDTH, HEIGHT), true);//открытие файла для записи
                 	if (!writering.isOpened()) {
                 	         std::cerr << "Failed to open output file" << std::endl;
                 	         return;
                 	            }

      cout << "124"<<endl;
                 		while (working)  { //цикл записи видео в файл

                                 do {
                                         FD_ZERO(&fds);
                                         FD_SET(fd, &fds);

                                         // задаем время ожидания доступности буфера
                                         tv.tv_sec = 2;
                                         tv.tv_usec = 0;//100

                                         // проверяем доступность буфера
                                         r = select(fd + 1, &fds, NULL, NULL, &tv);//проверка доступности буфера
                                 } while ((r == -1 && (errno = EINTR)));
                                 if (r == -1) {
                                         perror("select");
                                         return;
                                 }

                                 // захватываем мьютекс, охраняющий очередь с кадрами
                                                if (!qmutex.try_lock()) {
                                                    this_thread::sleep_for(50ms);
                                                }

                                                // удаляем "старые" кадры
                                                while (q.size() >= number_of_seconds * default_fps) {
                                                    q.pop_back();
                                                }

                 i++;
                 cout<<"KADR: "<< i << endl;
                                 CLEAR(buf);//макрос для очистки структуры buf от мусора и установки всех ее полей в 0.
                                 printf("dqbuf\n");
                                 buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                                 buf.memory = V4L2_MEMORY_MMAP;//установка способа доступа к буферу

                                 if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {

                                     if (errno == EAGAIN || errno == EIO) {
                                         // Возможно, буфер еще не готов или возникла ошибка ввода-вывода.
                                         // В этом случае пропустим итерацию и продолжим цикл.
                                         continue;
                                     } else {
                                         perror("Error dequeuing buffer");
                                         return;
                                     }
                                 }
                                    printf("memcpy\n");
                                     memcpy(val, buffers[buf.index].start, sizeof(char)*WIDTH*HEIGHT);//копирование содержимого буфера, на который указывает buf.index, в память по указателю val
                                     printf("memcpy proshel \n");

                         printf("zapis poshla\n");

                         cv::Mat frame(WIDTH,HEIGHT,CV_8UC1, val);
                         cv::Mat framer;
                         cv::cvtColor(frame, framer, cv::COLOR_BayerRG2BGR);
                         writering.write(framer);


                         if (framer.empty()) {//проверка на пустоту файла
                                     cerr << "ERROR! blank frame grabbed\n";
                                     break;
                                 }

                         if (ioctl(fd, VIDIOC_QBUF, &buf)==-1){// Помещаем буфер обратно в очередь захвата
                                     	   perror("Error dequeuing buffer");
                                     	                 return;
                                        }
                         // добавляем его в очередь
                                        q.emplace_front(move(framer));
                                        // разблокируем очередь
                                        qmutex.unlock();
                         printf("Kadr zapisan\n\n\n\n\n\n");


                 	}




                 	CLEAR(buf);
                 		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                 		buf.memory = V4L2_MEMORY_MMAP;

                 	        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                 	        xioctl(fd, VIDIOC_STREAMOFF, &type);//отключение потока видеозахвата
                 	        printf("xioctl\n");

                 	        for (i = 0; i < n_buffers; ++i){
                 	            munmap(buffers[i].start, buffers[i].length);//Эта строка используется для освобождения памяти, которая была выделена для буферов.
                 	            //Она вызывает функцию munmap для каждого буфера, которая освобождает ранее выделенную память.

                 	    }


                 	 printf("video saved\n");

                                 close(fd);
                                 delete[] buffers;
                                 printf("close fd\n");

                 cout << "[VIDEO_GRABBER] : stop" << endl;
             }









//блок отправки кадров по TCP/IP
      void send_queue(int sock) {
          lock_guard<mutex> lk(qmutex);
          uint32_t t = q.size();
      cout<<"start queue"<<endl;
          // пишем кол-во кадров
          send(sock, (char*) &t, sizeof(t), 0);
          // высоту
          t = grows;
          send(sock, (char*) &t, sizeof(t), 0);
          // и ширину кадра
          t = gcols;
          send(sock, (char*) &t, sizeof(t), 0);

          cv::VideoWriter videoWriterig;
              videoWriterig.open("out23.avi", cv::VideoWriter::fourcc('H','2','6','4'), default_fps, cv::Size(grows,gcols));
              cout<<"1"<<endl;
          int rc = 0;
          while (!q.empty()) {
          	cout<<"cikl"<<endl;
              // берем самый старый кадр
              Mat & ref_mat = q.back();
              cout<<"ref_mat"<<endl;
              // сохраняем кадр в файл
              videoWriterig.write(ref_mat);
                      cout<<"writer"<<endl;
              // вычисляем кол-во байт в нем
              uint32_t total = (ref_mat.dataend - ref_mat.data);
              // шлем это значение, чтобы клиент знал сколько
              // ему нужно вычитать байт этого кадра
              rc = send(sock, (char*) &total, sizeof(total), 0);
              int sent = 0;
              while (total > 0) {
                  // шлем непосредственно байты кадра
                  rc = send(sock, ref_mat.data + sent, total, 0);
                  if (rc == -1) break;
                  sent  += rc;
                  total -= rc;
              }
              // удаляем обработанный кадр из очереди
              q.pop_back();
          }
          // освобождаем ресурсы видеозаписи
          videoWriterig.release();
      }







int main() {


	//блок получения IP адреса по UDP
    int sockfd;
    char clientIP[INET_ADDRSTRLEN];
    bool ipReceived = false;
    struct sockaddr_in serverAddr, clientAddr;
    char buffer[BUFFER_SIZE];

    // Создаем сокет UDP
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // Настраиваем адрес сервера для прослушивания на всех интерфейсах и порту 3425
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(PORT);

    // Привязываем сокет к адресу сервера
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Binding failed");
        return 1;
    }

    while (!ipReceived) {

        // Получаем пакет
        socklen_t clientAddrLen = sizeof(clientAddr);
        ssize_t bytesRead = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
        if (bytesRead < 0) {
            perror("Receive failed");
            return 1;
        }

        // Преобразование полученных данных в строку
        std::string receivedData(buffer, bytesRead);
cout<<"+"<<endl;
        // Выводим содержимое пакета
      //  std::cout << "Received data: " << receivedData << std::endl;

        // Преобразование полученного IP-адреса из сетевого порядка байтов в строку

        inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIP, INET_ADDRSTRLEN);
        cout<<"- "<<endl;
        // Выводим IP-адрес клиента
        std::cout << "Client IP address: " << clientIP << std::endl;
        ipReceived=true;
    }

    // Закрыть сокет (этот код никогда не будет достигнут)
   // close(sockfd);



    // Блок подключения по TCP/IP
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        exit(1);
    }

    // IP и порт сервера
    unsigned short serverPort = 14100;

    // Создание структуры адреса сервера
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);

    if (inet_pton(AF_INET, clientIP, &(serverAddress.sin_addr)) <= 0) {
        perror("inet_pton");
        close(sock);
        return 1;
    }

    // Подключение к серверу
    if (connect(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        perror("connect");
        exit(1);
    }

    // Успешное подключение
    std::cout << "Connected to the server!" << std::endl;

/*
        for (;;) {
            // запускаем граббинг кадров в очередь
            // параллельно нашему треду
            working = 1;
            std::thread t(&video_grabber_thread);

            // а сами в этом треде ждем подключение клиента
            struct sockaddr_in client;
            socklen_t len = sizeof(struct sockaddr_in);
            int new_socket = accept(sock, (struct sockaddr *)&client, &len);

            // к нам подключились, тогда останавливаем граббинг видео
            working = 0;
            t.join();

            if (new_socket == -1) { // если подключения клиента нет, то...
                cerr << "ERR: new_socket = " << new_socket << endl;
                //break;
            } else { //если подключился клиент, то ..
                send_queue(new_socket);
            }
        }

        close(sock); // закрытие сокета
//сделать подключение к IP

*/

cout<<"Podklyschil k Mishe"<<endl;


        //блок чтения данных(команд) с байстаффинга
        //получение данных с сокета
        	while (true) {

        	//gpiod_line_set_value(gpio3_23, 1);  // Устанавливает значение GPIO-пина gpio3_23 равным 1.
        	//gpiod_line_set_value(gpio13, 0);   // Устанавливает значение GPIO-пина gpio13 равным 0.
        		//cout<<"Chectnie nachalos' "<<endl;
        	read_buffer[0] = 0x00;  // Устанавливает первый элемент массива read_buffer равным 0x00.

        	do {
        		num_bytes = 1;  // Устанавливает количество читаемых байт равным 1.
        		o = 0;
        		cout<<"Komanda polychena"<<endl;
        		cout<<endl;
        		cout<<endl;
        		do {
        			bytes_read = recv(sock, read_buffer, num_bytes, 0);
        			//bytes_read = read(serial_port, &read_buffer, num_bytes);  // Читает данные из последовательного порта в массив read_buffer.
        			//cout<<"Komanda polychena"<<endl;
        		} while ((read_buffer[0] != start_byte) & (bytes_read > 0));  // Продолжает цикл, пока первый элемент массива read_buffer не равен start_byte и количество прочитанных байт больше 0.

        		if (bytes_read > 0) {
        			get_buffer[0] = read_buffer[0];
        			printf("buffer[0] = %02X\n", get_buffer[0]);
        			printf("i = %d\n", o);
        			o++;
        			read_operation = 1;

        			while (read_operation != 0) {
        				o = 0;

        				switch (read_operation) {
        					case 1: {
        						num_bytes = 5;
        						count_bytes = 1;

        						while (num_bytes > 0) {
        							bytes_read = recv(sock, &buffer_bytestuff, 1,0);  // Читает данные из сокета в переменную buffer_bytestuff.
        							//gpiod_line_set_value(gpio8, value_gpio);  // Устанавливает значение GPIO-пина gpio8 равным значению переменной value_gpio.
        							//value_gpio = !value_gpio;  // Инвертирует значение переменной value_gpio.

        							if (bytes_read < 0)
        								break;

        							if (bytes_read > 0) {
        								if (buffer_bytestuff[0] == start_byte)
        									break;

        								if (buffer_bytestuff[0] != 0x1B) {
        									byte_bytestuff = buffer_bytestuff[0];
        								} else {
        									bytes_read = read(sock, &buffer_bytestuff, 1);
        									if (buffer_bytestuff[0] == start_byte)
        										break;
        									byte_bytestuff = buffer_bytestuff[0] - 0x20;
        								}

        								read_buffer[count_bytes++] = byte_bytestuff;
        								num_bytes--;
        							}
        						}

        						for (i = 1; i < count_bytes; i++) {
        							get_buffer[i] = read_buffer[i];
        						}

        						number_data = ((read_buffer[3] & 0x0F) * 0x100) + read_buffer[4] + 1 + 1;  // Вычисляет количество данных.
        						read_operation = 2;
        					} break;

        					case 2: {
        						num_bytes = number_data;
        						count_bytes = 6;

        						while (num_bytes > 0) {
        							bytes_read = recv(sock, &buffer_bytestuff, 1,0);  // Читает данные из последовательного порта в переменную buffer_bytestuff.

        							if (bytes_read < 0)
        								break;

        							if (bytes_read > 0) {
        								if (buffer_bytestuff[0] == start_byte)
        									break;

        								if (buffer_bytestuff[0] != 0x1B) {
        									byte_bytestuff = buffer_bytestuff[0];
        								} else {
        									bytes_read = recv(sock, &buffer_bytestuff, 1,0);
        									if (buffer_bytestuff[0] == start_byte)
        										break;
        									byte_bytestuff = buffer_bytestuff[0] - 0x20;
        								}

        								read_buffer[count_bytes++] = byte_bytestuff;
        								num_bytes--;
        							}
        						}

        						for (i = 6; i < count_bytes; i++) {
        							get_buffer[i] = read_buffer[i];
        							printf("buffer[%d] = %02X\n", i, get_buffer[i]);
        						}

        						read_operation = 0;
        					} break;

        					default: {
        						read_operation = 0;
        					} break;
        				}
        			}
        		}
cout << "KONES proxoda cikla"<<endl;
cout <<"BYTES_READ: "<<bytes_read<<endl;
        	} while ((read_buffer[0] != start_byte) && (bytes_read > 0));  // Продолжает цикл, пока первый элемент массива read_buffer не равен start_byte и количество прочитанных байт больше 0.
 //while (bytes_read > 0);
        	printf("END_GET\n");



cout << "CRC NACHALO POSHLO"<<endl;

        	//блок подсчета CRC полученного смс

        			CRC16 = calculation_crc(get_buffer, (count_bytes-2));//printf("CRC16 = %x\n", CRC16); то что подсчитали из полученного смс

        			CRC16_get = (get_buffer[count_bytes -2] << 8) | get_buffer[count_bytes-1];//printf("CRC16_get = %x\n", CRC16_get); то что получили из сообщения
        			val_from = get_buffer[1];
        			count_bytes = 0;

        			cout << "CRC16="<<CRC16<<endl;
        			cout << "CRC16_Get="<<CRC16_get<<endl;

        			// блок считывания команду в полученном смс
if (CRC16 == CRC16_get){ // если равны , то получено смс верно
			printf("CRC\n");

			//gpiod_line_set_value(gpio13, 1);
			//gpiod_line_set_value(gpio3_23, 1);
			//сделать проверку на сообщение таблица 7 (байт 5) 0xBB

			//блок управления командами, полученными в смс
				switch(get_buffer[8]){

				//блок команды 0хВ1
					case 0xB1: {//сделать стоп-кадр
						//Установка времени
						//Ввести проверку на NULL
						struct tm 				get_time = {0};
						if(get_buffer[4] == 0x0A){
							printf("buffer = %02d\n",  get_buffer[15]);
							printf("buffer = %02d\n",  get_buffer[14]);
							printf("buffer = %02d\n",  get_buffer[13]);
							printf("buffer = %02d\n",  get_buffer[10]);
							printf("buffer = %02d\n",  get_buffer[11]);
							printf("buffer = %02d\n",  get_buffer[12]);
							get_time.tm_sec = get_buffer[15];      	// секунды
							printf("start\n",  test_byte);
							get_time.tm_min = get_buffer[14];    		// минуты
							get_time.tm_hour = get_buffer[13];	   	// часы
							get_time.tm_mday = get_buffer[10];     	// день
							get_time.tm_mon = get_buffer[11]-1;		// месяц
							get_time.tm_year = get_buffer[12]+100;		// год

							new_time = mktime(&get_time);
							cout << "The local date and time is: " << ctime(&new_time) << endl;
							time_field.tv_sec = new_time;
							printf("time %ld\n", time_field.tv_sec);
							time_field.tv_usec=0;

							result_time = settimeofday(&time_field,NULL);
							if(result_time < 0){
								fprintf(stderr,"Error setting the Time.\n");
								return 0;
							}
						}



						printf("B1_ffff\n");
						//Делаем фото ???
						// создаем поток, который возвращает вектор данных фотографии
						make_photo_after_alarm = 1;
						status_thread = pthread_create(&photo_thread, NULL, &make_photo_video0, (void*)&photo_alarm);//создал
						if(status_thread != 0){
							printf("main error: can't create thread, status = %d\n", status_thread);
							exit(EXIT_FAILURE);
						}

						status_thread = pthread_join(photo_thread, NULL);//выполняешь
						if(status_thread != 0){
							printf("main error: can't join thread, status = %d\n", status_thread);
							exit(EXIT_FAILURE);
						}
						printf("FOTO+ \n");

						//Ответ ( подготовка буфера)
						translate_buffer[0] = start_byte;
						translate_buffer[1] = module_vk;
						translate_buffer[2] = val_from;
						translate_buffer[3] = 0x00;
						translate_buffer[4] = 0x03;
						translate_buffer[5] = 0xBB; // сообщение для камеры
						translate_buffer[6] = 0x00; //код участка
						translate_buffer[7] = 0x00;//номер камеры
						translate_buffer[8] = 0x0A; // код камеры
						CRC16 = calculation_crc(translate_buffer, 9);//printf("CRC16 = %x\n", CRC16);
						cout << "new CRC16="<<CRC16<<endl;
						translate_buffer[9] = CRC16 >> 8;
						translate_buffer[10] = CRC16 & 0x00FF;
						printf("Otvet formiryetsya \n");


						//байтстаффинг ответа
						i = 1;
						j = 1;
						output_buffer[0] = translate_buffer[0];
						while(i < 11){
							if(translate_buffer[i] == 0xA5){
								output_buffer[j] = 0x1B;
								j++;
								output_buffer[j] = 0xC5;
							} else {
								if(translate_buffer[i] == 0x1B){
									output_buffer[j] = 0x1B;
									j++;
									output_buffer[j] = 0x3B;
								} else {
									output_buffer[j] = translate_buffer[i];
								}
							}
							i++;
							j++;
						}


						for (step_preambula = 0; step_preambula < step_up; step_preambula++){
							send(sock, preambula, 1, 0);
						}
						for (i=0;i<j;i++){
							printf("Значение output_buffer: %02X\n", output_buffer[i]);
						}
						//printf("Значение output_buffer[0]: %02X\n", output_buffer[0]);
cout << "J="<<j<<endl;
						write(sock, &output_buffer[0], j);

/*
						if ((photo_alarm.size() != 0x00) || (photo_after_alarm.size() != 0x00)){
															for (step_preambula = 0; step_preambula < step_up; step_preambula++){
																send(sock, preambula, 1, 0);
															}

															write(sock, &output_buffer[0], j);
						}
*/
						printf("Otvet zapisyvaetsya \n");

						//очистка буферов
						memset(&get_buffer, '\0', sizeof(get_buffer));
						memset(&read_buffer, '\0', sizeof(read_buffer));
						memset(&translate_buffer, '\0', sizeof(translate_buffer));

						//ioctl(serial_port, TCIOFLUSH, &tty);
						//exit(EXIT_FAILURE);
						//free(translate_buffer);

						///СДЕЛАТЬ КАДРЫ
						//FF FF FF FF FF A5 0A 0B 00 03 BB 00 00 B1 DA 24 22
						//FF FF FF FF FF A5 0A 0B 00 05 BB 00 00 5A 00 00 4D 4F 22
					} break;


					//блок команды 0хС0
					case 0xC0: {
						printf("C0\n");
 						make_photo_stop = 1;
						printf("done = %d\n", photo_after_alarm_done);
						printf("size_photo = %ld\n", photo_alarm.size());
						//exit(EXIT_FAILURE);

						switch (rezhim){
							case 0x01:{
								rezhim = 0x02;
							} break;

							case 0x02:{
								translate_buffer[0] = start_byte;
								translate_buffer[1] = module_vk;
								translate_buffer[2] = val_from;
								translate_buffer[3] = 0x00;
								translate_buffer[4] = 0x14;// size 5 photo
								translate_buffer[5] = 0xBB;
								translate_buffer[6] = 0x00;
								translate_buffer[7] = 0x00;
								translate_buffer[8] = 0xC1;
								translate_buffer[9] = rezhim;

									translate_buffer[10] = 0x00; //номер группы фото // пока пусть будет 0

								translate_buffer[11] = 0x00;//photo_0_before_alarm.size() >> 16;;// наличие размера фото является указанием на наличие самой фото
								translate_buffer[12] = 0x00;//(photo_0_before_alarm.size() >> 8) & 0x00FF;
								translate_buffer[13] = 0x00;//photo_0_before_alarm.size() & 0x0000FF;
								translate_buffer[14] = 0x00;//photo_1_before_alarm.size() >> 16;;
								translate_buffer[15] = 0x00;//(photo_1_before_alarm.size() >> 8) & 0x00FF;
								translate_buffer[16] = 0x00;//photo_1_before_alarm.size() & 0x0000FF;
								translate_buffer[17] = 0x00;//photo_2_before_alarm.size() >> 16;;
								translate_buffer[18] = 0x00;//(photo_2_before_alarm.size() >> 8) & 0x00FF;
								translate_buffer[19] = 0x00;//photo_2_before_alarm.size() & 0x0000FF;
								translate_buffer[20] = photo_alarm.size() >> 16;
								translate_buffer[21] = (photo_alarm.size() >> 8) & 0x00FF;
								translate_buffer[22] = photo_alarm.size() & 0x0000FF;

								if (photo_after_alarm_done == 0){
									translate_buffer[23] = 0x00;
									translate_buffer[24] = 0x00;
									translate_buffer[25] = 0x00;
								}


								if (photo_after_alarm_done == 1){
									translate_buffer[23] = photo_after_alarm.size() >> 16;;
									translate_buffer[24] = (photo_after_alarm.size() >> 8) & 0x00FF;
									translate_buffer[25] = photo_after_alarm.size() & 0x0000FF;
								}


								CRC16 = calculation_crc(translate_buffer, 26);//printf("CRC166 = %x\n", CRC16);
								translate_buffer[26] = CRC16 >> 8;
								translate_buffer[27] = CRC16 & 0x00FF;
								 // конец подготовки к передаче
								i = 1;
								j = 1;
								output_buffer[0] = translate_buffer[0];
								while(i < 28){//проверка на байтстаффинг
									if(translate_buffer[i] == 0xA5){
										output_buffer[j] = 0x1B;
										j++;
										output_buffer[j] = 0xC5;
									} else {
										if(translate_buffer[i] == 0x1B){
											output_buffer[j] = 0x1B;
											j++;
											output_buffer[j] = 0x3B;
										} else {
											output_buffer[j] = translate_buffer[i];
										}
									}
									i++;
									j++;
								}


								if ((photo_alarm.size() != 0x00) || (photo_after_alarm.size() != 0x00)){
									for (step_preambula = 0; step_preambula < step_up; step_preambula++){
										send(sock, preambula, 1, 0);
									}

									write(sock, &output_buffer[0], j);//передача кадра по UART (ETHERNET)
								}
									memset(&get_buffer, '\0', sizeof(get_buffer));
									memset(&read_buffer, '\0', sizeof(read_buffer));
									memset(&translate_buffer, '\0', sizeof(translate_buffer));
									i = 0;
									j = 0;

									//узнать, что это такое ???
//посттревожный кадр через заданный интервал

									if(make_photo_after_alarm == 1){
										timer_photo.setTimeout([&]() {//функция таймера
											status_thread = pthread_create(&photo_thread, NULL, &make_photo_video0, (void*)&photo_after_alarm);
											if(status_thread != 0){
												printf("main error: can't create thread, status = %d\n", status_thread);
												exit(EXIT_FAILURE);
											}

											status_thread = pthread_join(photo_thread, NULL);
											if(status_thread != 0){
												printf("main error: can't join thread, status = %d\n", status_thread);
												exit(EXIT_FAILURE);
											}

											timer_photo.stop();
											photo_after_alarm_done = 1;
										}, alarm_interval);
										make_photo_after_alarm = 0;
									}
								//}

							} break;

							case 0x03:{
								rezhim = 0x02;
							}
						}

						//exit(EXIT_FAILURE);

					} break;


					//блок команды 0хВ3
				case 0xB3: {
    // Получение номера стоп-кадра и других параметров из полученного буфера
    number_out_data = get_buffer[13] * 256 * 256 + get_buffer[14] * 256 + get_buffer[15] + 1 + 3 + 3 + 3;
    number_stop_kadr = get_buffer[9];
    H_adress_begin_stop_kadr = get_buffer[10];
    M_adress_begin_stop_kadr = get_buffer[11];
    L_adress_begin_stop_kadr = get_buffer[12];
    start_kadr = H_adress_begin_stop_kadr * 256 * 256 + M_adress_begin_stop_kadr * 256 + L_adress_begin_stop_kadr;
    H_size_stop_kadr = get_buffer[13];
    M_size_stop_kadr = get_buffer[14];
    L_size_stop_kadr = get_buffer[15];
    size_kadr = H_size_stop_kadr * 256 * 256 + M_size_stop_kadr * 256 + L_size_stop_kadr;
    printf("photo_after_alarm_done = %d\n", photo_after_alarm_done);

    // Проверка на переполнение
    if (number_out_data > 0x0FFF) {
        printf("Переполнение\n");
        exit(EXIT_FAILURE);
        break;
    }


    // Заполнение буфера для передачи по RS485
    translate_buffer[0] = start_byte;
    translate_buffer[1] = module_vk;
    translate_buffer[2] = val_from;
    translate_buffer[3] = number_out_data >> 8;
    translate_buffer[4] = number_out_data & 0x00FF;
    translate_buffer[5] = 0xBB;
    translate_buffer[6] = 0x00;
    translate_buffer[7] = 0x00;
    translate_buffer[8] = 0xC3;
    translate_buffer[9] = number_stop_kadr;
    translate_buffer[10] = H_adress_begin_stop_kadr;
    translate_buffer[11] = M_adress_begin_stop_kadr;
    translate_buffer[12] = L_adress_begin_stop_kadr;
    translate_buffer[13] = H_size_stop_kadr;
    translate_buffer[14] = M_size_stop_kadr;
    translate_buffer[15] = L_size_stop_kadr;

    // Копирование данных стоп-кадра в буфер для передачи
    switch (number_stop_kadr) {
        case 4: {
            memcpy(translate_buffer + 16, &photo_alarm[start_kadr], size_kadr);
            //printf("memcopy 4\n");
        } break;

        case 5: {
            memcpy(translate_buffer + 16, &photo_after_alarm[start_kadr], size_kadr);
            //printf("memcopy 5\n");
        } break;

        default: {
            break;
            //memcpy(translate_buffer + 16, test_buffer + start_kadr, size_kadr);
            //memcpy(translate_buffer + 16, &photo_0_before_alarm[start_kadr], size_kadr);
            //printf("memcopy default\n");
        } break;
    }

    // Расчет контрольной суммы CRC16
    CRC16 = calculation_crc(translate_buffer, (size_kadr + 15 + 1));
    //printf("CRC16 = %x\n", CRC16);

    // Добавление контрольной суммы в буфер для передачи
    translate_buffer[(size_kadr + 15 + 1)] = CRC16 >> 8;
    translate_buffer[(size_kadr + 15 + 2)] = CRC16 & 0x00FF;

    i = 1;
    j = 1;
    output_buffer[0] = translate_buffer[0];

    // Замена байтов 0xA5 и 0x1B в буфере для передачи(байтстаффинг)
    while (i < ((size_kadr + 15 + 2) + 1)) {
        if (translate_buffer[i] == 0xA5) {
            output_buffer[j] = 0x1B;
            j++;
            output_buffer[j] = 0xC5;
            printf("increase a5\n");
        } else {
            if (translate_buffer[i] == 0x1B) {
                output_buffer[j] = 0x1B;
                j++;
                output_buffer[j] = 0x3B;
                printf("increase 1b\n");
            } else {
                output_buffer[j] = translate_buffer[i];
            }
        }

        i++;
        j++;
    }

    printf("write\n");
    printf("j = %d\n", j);

    // Запись преамбулы в последовательный порт
    for (step_preambula = 0; step_preambula < step_up; step_preambula++) {
    	write(sock, preambula, 1);
    }

    // Запись буфера в последовательный порт
    write(sock, &output_buffer[0], j);

    // Очистка буферов и переменных
    memset(&get_buffer, '\0', sizeof(get_buffer));
    memset(&read_buffer, '\0', sizeof(read_buffer));
    memset(&translate_buffer, '\0', sizeof(translate_buffer));
    i = 0;
    j = 0;

    printf("end write\n");

    // Проверка условия для очистки данных после передачи фото
    if ((number_stop_kadr == static_cast<int>(0x05)) && ((size_kadr + start_kadr) >= photo_after_alarm.size())) {
   // if ((number_stop_kadr == 0x05) && ((size_kadr + start_kadr) >= photo_after_alarm.size())) {
        make_photo_stop = 0;
        photo_0_before_alarm.clear();
        photo_1_before_alarm.clear();
        photo_2_before_alarm.clear();
        photo_alarm.clear();
        photo_after_alarm.clear();
        photo_after_alarm_done = 0;
        make_photo_after_alarm = 0;
        printf("clear\n");
        printf("clear done = %d\n", photo_after_alarm_done);
        //exit(EXIT_FAILURE);
    }
} break;

//блок остальных команд
default : {
	break;
}break;
}
}
						//}

}


    return 0;

}
