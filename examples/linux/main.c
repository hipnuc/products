#include <stdlib.h>
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 
#include <fcntl.h>
#include <errno.h>
#include <termios.h> 
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h> 
#include <pthread.h>
#include <signal.h>

#include "ch_serial.h"

#define ERRLOG(errmsg)  do{perror(errmsg);\
						   printf("%s--%s--%d\r\n", __FILE__, __FUNCTION__, __LINE__);\
						   exit(0);\
						  }while(0)

#define ARRAY_SIZE(x)  (sizeof(x) / sizeof((x)[0]))

#define SIZE   (sizeof(struct msgbuf) - sizeof(long))
#define MSG_BUF_SIZE		(128)
#define TYPE1  				(100)

static raw_t  raw;
static int    frame_rate;
static int    frame_count;
static int    cond_flag;
pthread_cond_t   cond_config, cond_rev;
pthread_mutex_t  lock_config, lock_rev;

struct msgbuf 
{
	long mtype;
	char buf[MSG_BUF_SIZE];
};

typedef struct
{
	uint8_t  code;
	char     name[8];
}item_code_name_t;

const item_code_name_t item_code_name[] = 
{
	{0x90, "id"      },
	{0xA0, "acc"     },
	{0xB0, "gyr"     },
	{0xC0, "mag"     },
	{0xD0, "eul"     },
	{0xD1, "quat"    },
	{0xF0, "pressure"},
	{0x91, "IMUSOL"  },
};

void dump_data_packet(raw_t *raw);

static const char *code2name(uint8_t code)
{
	const char *p = NULL;
	int i;
	for(i = 0; i < ARRAY_SIZE(item_code_name); i++)
	{
		if(code == item_code_name[i].code)
			p = item_code_name[i].name;
	}

	return p;
}

void *pthread_msgrecv(void *arg)
{
	int fd = *((int *)arg);
	int msqid;
	struct msgbuf msg_rcv;
	char eout[] = "AT+EOUT=1\r\n";
	
	key_t key;

	if((key = ftok("./", 'b')) == -1)
		ERRLOG("ftok error");

 	if((msqid = msgget(key, IPC_CREAT | IPC_EXCL | 0664)) == -1)
	{
		if(errno != EEXIST)
		{
			ERRLOG("msgget error");	
		}
		else
			msqid = msgget(key, 0664);
	}

	while(1)
	{
		msgrcv(msqid, &msg_rcv, SIZE, TYPE1 , 0);

		cond_flag = 1;

		if(strncmp(msg_rcv.buf, "quit", 4) == 0)
		{
			cond_flag = 0;
			pthread_cond_signal(&cond_config);
			write(fd, eout, sizeof(eout));
			continue;
		}

		printf("send_cmd:%s\n", msg_rcv.buf);
		write(fd, &msg_rcv.buf, strlen(msg_rcv.buf));
		memset(&msg_rcv.buf, 0, strlen(msg_rcv.buf));

		sleep(1);
	}
	pthread_exit(0);
}

void *pthread_imudata(void *arg)
{
	int n = 0;
	int rev = 0;
	int count = 0;
	int fd = *((int *)arg);	
	uint8_t buf[2048] = "";
	
	alarm(1);

	while(1)
	{
		if(cond_flag)
		{
			pthread_cond_signal(&cond_rev);
			pthread_mutex_lock(&lock_config);
			pthread_cond_wait(&cond_config, &lock_config);
			pthread_mutex_unlock(&lock_config);
			cond_flag = 0;
		}

		n = read(fd, buf, sizeof(buf));
		count += n;
		if(n > 0)
		{
			for(int i=0; i < n; i++)
			{
				rev = ch_serial_input(&raw, buf[i]);

				if(rev)
				{
					count = 0;
					
					frame_count++;
					if(!(frame_count % 10))
					{
						puts("\033c");
						dump_data_packet(&raw);
					}
				}
			}
		}
	}	

	pthread_exit("th2");
}

void *pthread_putinfo(void *arg)
{
	int fd = *((int *)arg);
	int n = 0;
	char rev_buf[128] = "";

	while(1)
	{
		if(!cond_flag)
		{
			pthread_mutex_lock(&lock_rev);
			pthread_cond_wait(&cond_rev, &lock_rev);
			pthread_mutex_unlock(&lock_rev);
		}

		n = read(fd, rev_buf, sizeof(rev_buf));
		for(int i = 0; i < n; i++)
			putchar(rev_buf[i]);
		memset(rev_buf, 0, sizeof(rev_buf));
	}
}


int open_port(char *port_device, int baud)
{
	struct termios options;
	int fd = open(port_device, O_RDWR | O_NOCTTY);
	
	tcgetattr(fd, &options);

	if (fd == -1)
	{
		perror("open_port: Unable to open SerialPort");
		puts("Please check the usb port name!!!");
		puts("such as \" sudo ./main ttyUSB0 \"");
		exit(0);
	}

	if(fcntl(fd, F_SETFL, 0) < 0)
	{
		printf("fcntl failed\n");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
  
	if(isatty(STDIN_FILENO) == 0)
	{
		printf("standard input is not a terminal device\n");
	}
	else 
	{
		printf("isatty success!\n");
	}


	bzero(&options, sizeof(options));

	options.c_cflag = CS8 | CLOCAL | CREAD;

	switch(baud)
	{
		case 9600:
			options.c_cflag |= B9600;
			break;
		case 115200:
			options.c_cflag |= B115200;
			break;
		case 230400:
			options.c_cflag |= B230400;
			break;
		case 460800:
			options.c_cflag |= B460800;
			break;
		case 921600:
			options.c_cflag |= B921600;
			break;
		default:
			printf("port baud input error!\r\n");
			exit(0);
	}


	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	return (fd);
}

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}


int main(int argc, const char *argv[])
{
	int fd = 0, baud = 0;
	char dir_usb_dev[64] = "/dev/";

	int i;
	pthread_t thread1, thread2,thread3;

	signal(SIGALRM, timer);
	if(argc > 2)
	{
		strcat(dir_usb_dev, argv[1]);
		sscanf(argv[2], "%d", &baud);
		fd = open_port(dir_usb_dev, baud);
	}
	else
	{
		puts("Please enter USB port append to the execution command!!!");
		exit(0);	
	}

	if(pthread_mutex_init(&lock_config, NULL) != 0)
		ERRLOG("pthread_mutex_init error");

	if(pthread_cond_init(&cond_config, NULL) != 0)
		ERRLOG("pthread_cond_init error");

	if(pthread_mutex_init(&lock_rev, NULL) != 0)
		ERRLOG("pthread_mutex_init error");

	if(pthread_cond_init(&cond_rev, NULL) != 0)
		ERRLOG("pthread_cond_init error");

	if(pthread_create(&thread1, NULL, pthread_msgrecv, (void *)&fd) != 0)
		ERRLOG("pthread_create error");

	if(pthread_create(&thread2, NULL, pthread_imudata, (void *)&fd) != 0)
		ERRLOG("pthread_create error");

	if(pthread_create(&thread3, NULL, pthread_putinfo, (void *)&fd) != 0)
		ERRLOG("pthread_create error");

	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread3, NULL);


	pthread_cond_destroy(&cond_config);

	pthread_mutex_destroy(&lock_config);

	close(fd);
}

void dump_data_packet(raw_t *raw)
{
	int i;
	printf("%-16s%8dHz\r\n",   "frame rate:", frame_rate);
	printf("%-16s%8.3f %8.3f %8.3f\r\n",   "acc(G):",     raw->imu.acc[0], raw->imu.acc[1], raw->imu.acc[2]);
	printf("%-16s%8.3f %8.3f %8.3f\r\n",   "gyr(deg/s):", raw->imu.gyr[0], raw->imu.gyr[1], raw->imu.gyr[2]);
	printf("%-16s%8.3f %8.3f %8.3f\r\n",   "mag(uT):",    raw->imu.mag[0], raw->imu.mag[1], raw->imu.mag[2]);
	printf("%-16s%8.3f %8.3f %8.3f\r\n",   "eul(deg):",   raw->imu.eul[0], raw->imu.eul[1], raw->imu.eul[2]);
	printf("%-16s%8.3f %8.3f %8.3f %8.3f\r\n",   "quat;",  raw->imu.quat[0], raw->imu.quat[1], raw->imu.quat[2], raw->imu.quat[3]);
	printf("%-16s%8.3f\r\n",     "presure(pa):",  raw->imu.pressure);
	printf("%-16s%8d\r\n",       "timestamp(ms):", raw->imu.timestamp);
	printf("packet: ");
	for(i = 0; i < raw->nitem_code; i++)
	{
		printf("0x%02X(%s)", raw->item_code[i], code2name(raw->item_code[i]));
	}
		
	printf("\r\n");
}
			
