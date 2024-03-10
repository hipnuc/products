#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "serial_port.h"
#include "hipnuc.h"

#define REFRESH_INTERVAL 20000 // 以微秒为单位，对应于50Hz


int main(int argc, char *argv[])
{
	hipnuc_raw_t hipnuc_raw = {0};
	int baud_rate = 115200;
	uint8_t recv_buf[2048];
	char log[512];
	int len;

	if (argc < 2)
	{
		fprintf(stderr, "Usage: %s <serial_port> [baud_rate]\n", argv[0]);
		return 1;
	}

	char *port_name = argv[1];

	if (argc > 2)
		baud_rate = atoi(argv[2]);

	int fd = serial_port_open(port_name);
	if (fd < 0)
		return 1;

	if (serial_port_configure(fd, baud_rate) < 0)
	{
		fprintf(stderr, "Cannot open %s\n", port_name);
		serial_port_close(fd);
		return 1;
	}

	printf("* %s successfully open with %d.\n", port_name, baud_rate);
	printf("* Starting serial data reader. Press CTRL+C to exit.\n");
	printf("* Select mode:\n");
	printf("  R - Read device data\n");
	printf("  C - Send command to device\n");
	printf("* Enter your choice: ");
	int choice = getchar();

	if (choice == 'R' || choice == 'r')
	{
		/* enable output */
		serial_send_then_recv(fd, "AT+EOUT=1\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);
		printf("Being read data...\n");
		while (1)
		{
			len = read(fd, recv_buf, sizeof(recv_buf));

			for (int i = 0; i < len; i++)
			{
				if (hipnuc_input(&hipnuc_raw, recv_buf[i]))
				{
				    hipnuc_dump_packet(&hipnuc_raw, log, sizeof(log));
					printf("\033[H\033[J");
					printf("%s", log);
				}
			}
			usleep(REFRESH_INTERVAL);
		}
	}

	else if (choice == 'C' || choice == 'c')
	{
		/* stop data output */
		serial_send_then_recv(fd, "AT+EOUT=0\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

		printf("Enter command(example: AT+INFO) or Press CTRL+C to exit:\n");

		/* clear buffer */
		int ch;
		while ((ch = getchar()) != '\n' && ch != EOF)
			;

		fgets(log, sizeof(log), stdin);

		len = serial_send_then_recv(fd, log, "", recv_buf, sizeof(recv_buf), 500);
		if (len > 0)
		{
			printf("Received %d bytes: \n\n", len);

			for (int i = 0; i < len; i++)
			{
				if (recv_buf[i] == '\r' || recv_buf[i] == '\n')
				{
					printf("%c", recv_buf[i]);
				}
				else
				{
					printf("%c", isprint(recv_buf[i]) ? recv_buf[i] : '.');
				}
			}
		}
		else
		{
			printf("No response or error.\n");
		}
	}
	else
	{
		printf("Invalid choice.\n");
	}

	return 0;
}

