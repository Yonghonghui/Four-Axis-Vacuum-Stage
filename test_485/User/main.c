#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/RS485/rs485.h"
#include "Emm.h"

// struct Motor {
//     int reduction_ratio; 
//     uint8_t acc = 50; 
//     uint16_t tgt_velocity; //[RPM]; 42motors:300, 28motors:
//     uint32_t tgt_position; //[/65536 = round]
//     double tgt_degree; // [degree]

//     uint16_t read_velocity; //[RPM]; 42motors:300, 28motors:
//     uint32_t read_position; //[/65536 = round]
//     double read_degree; // [degree]
// } motor0, motor1, motor2, motor3;

// void Motor_Param_Refreash(*Motor motor) {
//     motor.read_degree = motor.read_position * 360 / (65536 * motor.reduction_ratio); // [degree]
//     motor.tgt_position = motor.tgt_degree * (65536 * motor.reduction_ratio) / 360; // [/65536 = round]
// }

void Translate_received_data(uint8_t* rs485buf){
    uint8_t len;
    uint8_t motor_addr;
    uint8_t function_code;
    rs485_receive_data(rs485buf, &len);
    if (len)    // data length !=0;
        {
            if (len > 8) {
                len = 8;
            }
            motor_addr = rs485buf[0]; // read out the address
            function_code=rs485buf[1]; // read out the type of received data

            switch(function_code){
                case 0x1F: break;// 读取固件版本和对应的硬件版本
                case 0x20: break;//读取相电阻和相电感
                case 0x21: break;//读取位置环PID参数
                case 0x24: break;//读取总线电压
                case 0x27: break;//读取相电流
                case 0x31: break;//读取经过线性化校准后的编码器值
                case 0x33: break;//读取电机目标位置
                case 0x35:  /* 处理电机实时转速 */
                {
                    /* 校验数据长度是否符合预期（地址1 + 功能码1 + 符号1 + 转速2 + 校验1 = 6字节） */
                    if (len >= 6)
                    {
                        uint8_t sign = rs485buf[2];  /* 符号位：0x00正，0x01负 */
                        uint16_t speed_raw = (rs485buf[3] << 8) | rs485buf[4];  /* 组合转速值 */
                        int32_t speed_rpm = (sign == 0x01) ? -speed_raw : speed_raw;  /* 计算有符号转速 */
                        
                        /* 示例：通过串口打印转速 */
                        printf("Motor [%02X] Speed: %d RPM\n", motor_addr, speed_rpm);
                        
                        /* 示例：在LCD显示转速（假设坐标30,210） */
                        lcd_show_num(30, 300, speed_rpm, 5, 16, BLUE);//电机实时转速 = -0x05DC = -1500RPM（转/每分钟）
                    }
                    else
                    {
                        /* 数据长度不足，处理错误 */
                        printf("Error: Invalid data length for speed!\n");
                    }
                    break;
                }
                case 0x36: ///* 处理电机实时位置 */
                {
                    /* 校验数据长度是否符合预期（地址1 + 功能码1 + 符号1 + 位置4 + 校验1 = 8字节） */
                    if (len >= 8)
                    {
                        uint8_t sign = rs485buf[2];  /* 符号位：0x00正，0x01负 */
                        
                        /* 提取4字节位置值（大端模式） */
                        uint32_t position_raw = ((uint32_t)rs485buf[3] << 24) |
                                               ((uint32_t)rs485buf[4] << 16) |
                                               ((uint32_t)rs485buf[5] << 8)  |
                                               (uint32_t)rs485buf[6];
                        
                        /* 转换为有符号值 */
                        int32_t position_signed = (sign == 0x01) ? -position_raw : position_raw;
                        
                        /* 计算角度（示例公式：position_raw对应360°） */
                        float angle_deg = (position_signed * 360.0f) / 65536.0f;
                        
                        /* 示例：通过串口打印位置和角度 */
                        printf("Motor [%02X] Position: Raw=%ld, Angle=%.2f°\n", 
                               motor_addr, position_signed, angle_deg);
                        
                        /* 示例：在LCD显示角度（假设坐标30,210） */
                        lcd_show_xnum(30, 210, (int32_t)angle_deg, 5, 16, 0x80, BLUE);
                    }
                    else
                    {
                        /* 数据长度不足，处理错误 */
                        printf("Error: Invalid data length for position!\n");
                    }
                    break;
                }
                case 0x37: break;//读取电机位置误差
                case 0x3A: break;//读取电机状态标志位
                case 0x3B: break;//读取回零状态标志位
                case 0x42: break;//读取驱动配置参数
                case 0x43: break;//读取系统状态参数
                default: break;
            }
        //     for (i = 0; i < len; i++) {
        //         lcd_show_xnum(30 + i * 32, 210, rs485buf[i], 3, 16, 0x80, BLUE);  
        //     }
        }

}

int main(void)
{
    uint8_t key;
    //uint8_t i = 0, 
    uint8_t t = 0;
    uint8_t cnt = 0;
    uint8_t rs485buf[8];
    // uint8_t len = 0;

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
    // motor0.reduction_ratio = 100;
    // motor0.acc = 10;
    // motor1.reduction_ratio = 30;
    // motor2.reduction_ratio = 10;
    uint8_t acc = 50;
    uint16_t vel = 10;

    while(1) //main loop
    {
        key = key_scan(0);

        if(key == KEY0_PRES){
			lcd_show_string(30, 150, 500, 100, 16, "Main Sequence ", BLUE);
            // motor0.tgt_degree = 15;
            // motor1.tgt_degree = 900; //[mm]
            Emm_V5_Vel_Control(1, 0, vel, acc, 0);
            
        }

        if(key==KEY1_PRES){
            vel -= 5;
            lcd_show_string(30, 170, 500, 100, 16, "Vel --, Vel= ", BLUE);
					
            lcd_show_num(30 + 12 * 8, 170, vel, 5, 16, BLUE);

        }
        if(key==KEY2_PRES){

        }

        Emm_V5_Read_Sys_Params(1, S_CPOS);
        delay_ms(100);
        // rs485_receive_data(rs485buf, &len);
        Translate_received_data(rs485buf);

        t++;
        delay_ms(10);

        if (t == 20)
        {
            LED0_TOGGLE();  /* LED0��˸, ��ʾϵͳ�������� */
            t = 0;

            cnt++;
            lcd_show_xnum(30 + 48, 130, cnt, 3, 16, 0x80, BLUE);    /* ��ʾ���� */
        }
    } //main loop
}


