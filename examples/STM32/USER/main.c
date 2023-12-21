#include "delay.h"
#include "usart.h"
#include "ch_serial.h"
 
/************************************************
 ALIENTEK战舰STM32开发板实验
 串口实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

/************************************************
 北京超核电子
 惯性导航模块： HI226 HI229
 串口接收数据例程
 本例程只供学习使用，观察数据输出，不做其他用途
 串口2接收来自HI226或者是HI229的数据
 串口1将串口2成功接收到的数据打印到终端上
 这里的终端一般指的是PC机上串口调试助手
 官网：http://www.hipnuc.com
************************************************/


static raw_t raw = {0};             /* IMU stram read/control struct */
static uint8_t decode_succ;         /* 0: no new frame arrived, 1: new frame arrived */


int main(void)
{		
    delay_init();	    	                               //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);        //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    uart_init(115200);	                                   //串口初始化为115200

    while(1)
    {
        if(decode_succ)
        {
            decode_succ = 0;
            delay_ms(800);
            ch_dump_imu_data(&raw);
        }
	}	 
}

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
