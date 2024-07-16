#!/bin/bash
#tty=/dev/ttyUSB0

if [ $# -ne 2 ];then
	echo "请提供正确的参数：串口号 config"
	echo "例如：sh hipnuc._con.sh /dev/ttyUSB0 config.yaml"
	exit 1
fi

SERIAL_PORT=$1
CONFIG_FILE=$2

if ! [ -e $SERIAL_PORT ];then
	echo "串口设备 $SERIAL_PORT 不存在"
	exit 1
fi

echo "IMU port: " $SERIAL_PORT
echo "CONFIG FILE:" $CONFIG_FILE
fuser -k $SERIAL_PORT

TEMP_FILE=$(mktemp)

BAUD_RATES=(9600 115200 230400 460800 921600)

detect_baud_rate() {
	for BAUD in "${BAUD_RATES[@]}"; do
		stty -F $SERIAL_PORT $BAUD cs8 -cstopb -parenb
		echo "start $BAUD"
		for i in {1..5}; do
			(cat $SERIAL_PORT > $TEMP_FILE & echo -e "AT+EOUT=0\r\n" > $SERIAL_PORT)
			sleep 0.1
			if grep -iq "OK" $TEMP_FILE; then
				echo "CONNECT SUCCESS"
				echo "Detected baud rate: $BAUD"
				return 0
			else
				echo "SCMD AT+EOUT=0 ERROR"
			fi
		done
	done
	return 1
}

if ! detect_baud_rate; then
	echo "Failed to detect baud rate!"
	rm "$TEMP_FILE"
	exit 1
fi

if [ ! -f "$TEMP_FILE" ]; then
	echo "Failed to create temporary file"
	exit 1
fi
> $TEMP_FILE

echo "AT+INFO"
(cat $SERIAL_PORT > $TEMP_FILE & echo -e "AT+INFO\r\n" > $SERIAL_PORT)
sleep 0.1
response=$(cat $TEMP_FILE | tr -d '\000' | sed '/^$/d')
echo "$response"

if grep -iq "ok" $TEMP_FILE; then
	echo " "
	echo "start config IMU"
else
	echo "SCMD AT+INFO ERROR"
	exit
fi

echo " "

commands=$(awk '/commands:/{flag=1;next} /:/{flag=0} flag {gsub("- ", ""); print}' $CONFIG_FILE | tr -d '\000')
echo "$commands" | while IFS= read -r command; do 
	>$TEMP_FILE
	echo "SCMD: ${command}"
	(cat $SERIAL_PORT > $TEMP_FILE & echo -e "${command}\r\n" > $SERIAL_PORT)

	sleep 0.1

	if grep -iq "ok" $TEMP_FILE; then
		response=$(cat $TEMP_FILE | tr -d '\000' | sed '/^$/d')
		echo "REV:	$response"
	else
		echo "error"
		break
	fi
done

if ! detect_baud_rate; then
	echo "Failed to detect baud rate!"
	rm "$TEMP_FILE"
	exit 1
fi

(cat $SERIAL_PORT > $TEMP_FILE & echo -e "AT+INFO\r\n" > $SERIAL_PORT)
sleep 0.1
response=$(cat $TEMP_FILE | tr -d '\000' | sed '/^$/d')
echo "$response"

if [ -f "$TEMP_FILE" ]; then
	rm $TEMP_FILE
else
	echo "Temporary file not found."
fi

echo "Done!!!"
exit 0 
