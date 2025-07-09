#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/RS485/rs485.h"
#include "Emm.h"

int main(void)
{
    uint8_t key;
    uint8_t i = 0, t = 0;
    uint8_t cnt = 0;
    uint8_t rs485buf[8];

    HAL_Init();                          
    sys_stm32_clock_init(336, 8, 2, 7);     /* 168Mhz */
    delay_init(168);
    usart_init(115200);
    usmart_dev.init(84);
    led_init();
    lcd_init();
    key_init();
    rs485_init(115200);

    lcd_show_string(30,  50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30,  70, 200, 16, 16, "RS485 TEST", RED);
    lcd_show_string(30,  90, 200, 16, 16, "ATOM@ALIENTEK", RED);
    lcd_show_string(30, 110, 200, 16, 16, "KEY0:Send", RED);    

    lcd_show_string(30, 130, 200, 16, 16, "Count:", RED);      
    lcd_show_string(30, 150, 200, 16, 16, "Send Data:", RED);   
    lcd_show_string(30, 190, 200, 16, 16, "Receive Data:", RED);

    // motor parameters
    uint8_t acc = 50;
    uint16_t vel = 300;

    while(1) //main loop
    {
        key = key_scan(0);

        if(key == KEY0_PRES){
			vel = 5;
            lcd_show_string(30, 170, 500, 100, 16, "Mod 1, Vel= 5    ", BLUE);
					
            Emm_V5_Vel_Control(1, 0, vel, acc, 0);
            //Emm_V5_Pos_Control(1, 0, vel, acc, 9600, 0, 0);
            //delay_ms(100);
            //Emm_V5_Pos_Control(2, 0, 300, 0, 6400, 0, 0);
        }

        if(key==KEY1_PRES){
            vel -= 5;
            lcd_show_string(30, 170, 500, 100, 16, "Vel --, Vel= ", BLUE);
					
            lcd_show_num(30 + 12 * 8, 170, vel, 5, 16, BLUE);
            Emm_V5_Vel_Control(1, 0, vel, acc, 0);
            delay_ms(100);
            // Emm_V5_Read_Sys_Params(1, S_TPOS);
        }
        if(key==KEY2_PRES){
            vel += 5;
            lcd_show_string(30, 170, 500, 100, 16, "Vel ++, Vel= ", BLUE);
					
            lcd_show_num(30 + 12 * 8, 170, vel, 5, 16, BLUE);
            Emm_V5_Vel_Control(1, 0, vel, acc, 0);
            delay_ms(100);
            Emm_V5_Read_Sys_Params(1, S_TPOS);    
        }

        rs485_receive_data(rs485buf, &key);
        if (key)    /* ���յ������� */
        {
            if (key > 8) {
                key = 8;  
            }

            for (i = 0; i < key; i++) {
                lcd_show_xnum(30 + i * 32, 210, rs485buf[i], 3, 16, 0x80, BLUE);  
            }
        }

        t++;
        delay_ms(10);

        if (t == 20)
        {
            LED0_TOGGLE();  /* LED0��˸, ��ʾϵͳ�������� */
            t = 0;

            cnt++;
            lcd_show_xnum(30 + 48, 130, cnt, 3, 16, 0x80, BLUE);  
        }
    } //main loop
}


