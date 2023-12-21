all:main config_imu
.PHONY:all

main:ch_serial.o main.o
	gcc -o main $^ -lpthread

config_imu:config_imu.o
	gcc -o config_imu $^

%.o:%.c
	gcc -c -o $@ $<

.PHONY:clean
clean:
	rm *.o main config_imu

