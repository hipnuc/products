#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>  //sleep 

#define SIZE sizeof(struct msgbuf) - sizeof(long)
#define N 128
#define TYPE1 (100)
#define TYPE2 (200)
struct msgbuf {
	long mtype;
	char buf[N];
};

int main(int argc, const char *argv[])
{
	char stop_frame[] = "AT+EOUT=0\r\n";
	key_t key;

	if((key = ftok("./", 'b')) < 0)
	{
		perror("ftok error");
		return -1;
	}

	int msqid;
	struct msgbuf msg_snd;
	if((msqid = msgget(key, IPC_CREAT | IPC_EXCL | IPC_NOWAIT | 0664)) < 0)
	{
		if(errno != EEXIST)
		{
			perror("msgget error");
			return -1;	
		}
		else
			msqid = msgget(key, 0664);
	}

	msg_snd.mtype = TYPE1;
	printf("ok\r\n");
	memcpy(&msg_snd.buf, stop_frame, sizeof(stop_frame));
	msgsnd(msqid, &msg_snd, SIZE, 0);

	while(1)
	{
		fgets(msg_snd.buf, N, stdin);
		msg_snd.buf[strlen(msg_snd.buf) - 1] = '\r';
		msg_snd.buf[strlen(msg_snd.buf)] = '\n';
		msg_snd.buf[strlen(msg_snd.buf) + 1] = '\0';
		
		msgsnd(msqid, &msg_snd, SIZE, 0);
		if(strncmp(msg_snd.buf, "quit", 4) == 0)
			exit(0);
		memset(&msg_snd.buf, 0, sizeof(msg_snd.buf));

	}


	return 0;
}
