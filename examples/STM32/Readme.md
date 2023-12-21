# STM32例程

​    本例程提供了C 语言例程代码

​	测试环境：Windows10_x64 操作系统

​	编译器：keil_V5.28

​	开发板：正点原子-战舰V3 STM32F103ZET6

​	测试设备：HI226/HI229/CH100/CH020/CH040

### 硬件连接

1. 将Hi226/Hi229 正确插入到模块评估板上。

| 超核调试板 | 正点原子开发板 |
| ---------- | -------------- |
| RXD        | PA2(TXD)       |
| TXD        | PA3(RXD)       |
| 3.3V       | 3V3            |
| GND        | GND            |

2. 用杜邦线将上表相对应的引脚连接起来。*
3. 用USB线插到开发板的 __USB_232__ 插口上，另一端插到电脑上。

### 观察输出

​	打开串口调试助手，打开开发板对应的串口号，观察数据输出

```
id:             1
frame rate:     100
acc(G):         0.125 0.840 0.306
gyr(deg/s):     -0.471 -0.186 0.029
mag(uT):        -0.767 24.183 39.417
eul(deg):       -21.661 67.983 0.018
quat;           0.814 0.549 -0.156 -0.105
presure(pa):    0.000
timestamp(ms):  8452
item: 0x91(IMUSOL)
```

### 代码结构

解码程序部分: 

* ch_serial.c文件用于解析模块输出的数据协议。

使用说明:

1. 先确硬件连接正确，并且有基本的MCU串口操作知识，使用过串口中断接收

2. 初始化串口

   ```c
       uart_init(115200);	
   ```

3. 在对应的串口中断中调用解析函数接口ch_serial_input（）

    ```c
    void USART2_IRQHandler(void)
    {
    	uint8_t ch;
    	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
        {
            ch = USART_ReceiveData(USART2);	
        }
        
        /* decode each byte */
        decode_succ = ch_serial_input(&raw, ch);
    } 
    ```

4. 在main函数中(while(1)) 中调用打印函数，打印接收到的数据

   ```c
   	while(1)
       {
           if(decode_succ)
           {
               decode_succ = 0;
               delay_ms(800);
               dump_imu_data(&raw);  /* 打印函数 */
           }
   	}
   ```
   
   

