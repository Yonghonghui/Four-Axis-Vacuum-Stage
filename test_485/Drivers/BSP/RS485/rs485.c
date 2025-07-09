/**
 ****************************************************************************************************
 * @file        rs485.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 7* @date        2021-10-27
 * @brief       RS485 ��������
 * @license     Copyright (c) 2020-2032, �������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨������ԭ�� F407���������
 * ������Ƶ��www.yuanzige.com
 * ������̳��http://www.openedv.com/forum.php
 * ��˾��ַ��www.alientek.com
 * �����ַ��zhengdianyuanzi.tmall.com
 *
 * �޸�˵��
 * V1.0 20211027
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/RS485/rs485.h"
#include "./SYSTEM/delay/delay.h"


UART_HandleTypeDef g_rs458_handler;     /* RS485���ƾ��(����) */

#ifdef RS485_EN_RX /* ���ʹ���˽��� */

uint8_t g_RS485_rx_buf[RS485_REC_LEN]; /* ���ջ���, ��� RS485_REC_LEN ���ֽ�. */
uint8_t g_RS485_rx_cnt = 0;            /* ���յ������ݳ��� */

void RS485_UX_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs458_handler, UART_FLAG_RXNE) != RESET)) /* ���յ����� */
    {
        HAL_UART_Receive(&g_rs458_handler, &res, 1, 1000);

        if (g_RS485_rx_cnt < RS485_REC_LEN)         /* ������δ�� */
        {
            g_RS485_rx_buf[g_RS485_rx_cnt] = res;   /* ��¼���յ���ֵ */
            g_RS485_rx_cnt++;                       /* ������������1 */
        }
    }
}

#endif

/**
 * @brief       RS485��ʼ������
 *   @note      �ú�����Ҫ�ǳ�ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @retval      ��
 */
void rs485_init(uint32_t baudrate)
{

    /* IO �� ʱ������ */
    RS485_TX_GPIO_CLK_ENABLE(); /* ʹ�� ����TX�� ʱ�� */
    RS485_RX_GPIO_CLK_ENABLE(); /* ʹ�� ����RX�� ʱ�� */
    RS485_UX_CLK_ENABLE();      /* ʹ�� ���� ʱ�� */

    GPIO_InitTypeDef gpio_initure;
    gpio_initure.Pin = RS485_TX_GPIO_PIN;
    gpio_initure.Mode = GPIO_MODE_AF_PP;
    gpio_initure.Pull = GPIO_PULLUP;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_initure.Alternate = GPIO_AF7_USART3;               /* ����Ϊ����3 */
    HAL_GPIO_Init(RS485_TX_GPIO_PORT, &gpio_initure);       /* ����TX �� ģʽ���� */

    gpio_initure.Pin = RS485_RX_GPIO_PIN;
    HAL_GPIO_Init(RS485_RX_GPIO_PORT, &gpio_initure);       /* ����RX �� �������ó�����ģʽ */

    /* USART ��ʼ������ */
    g_rs458_handler.Instance = RS485_UX;                    /* ѡ��485��Ӧ�Ĵ��� */
    g_rs458_handler.Init.BaudRate = baudrate;               /* ������ */
    g_rs458_handler.Init.WordLength = UART_WORDLENGTH_8B;   /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_rs458_handler.Init.StopBits = UART_STOPBITS_1;        /* һ��ֹͣλ */
    g_rs458_handler.Init.Parity = UART_PARITY_NONE;         /* ����żУ��λ */
    g_rs458_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* ��Ӳ������ */
    g_rs458_handler.Init.Mode = UART_MODE_TX_RX;            /* �շ�ģʽ */
    HAL_UART_Init(&g_rs458_handler);                        /* ʹ�ܶ�Ӧ�Ĵ���, ����Msp */
    __HAL_UART_DISABLE_IT(&g_rs458_handler, UART_IT_TC);

#if RS485_EN_RX /* ���ʹ���˽��� */
                /* ʹ�ܽ����ж� */
    __HAL_UART_ENABLE_IT(&g_rs458_handler, UART_IT_RXNE);   /* ���������ж� */
    HAL_NVIC_EnableIRQ(RS485_UX_IRQn);                      /* ʹ��USART1�ж� */
    HAL_NVIC_SetPriority(RS485_UX_IRQn, 3, 3);              /* ��ռ���ȼ�3�������ȼ�3 */
#endif

}

/**
 * @brief       RS485����len���ֽ�
 * @param       buf     : �������׵�ַ
 * @param       len     : ���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ���� RS485_REC_LEN ���ֽ�)
 * @retval      ��
 */
void rs485_send_data(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(&g_rs458_handler, buf, len, 1000); /* ����2�������� */
    g_RS485_rx_cnt = 0;
}

/**
 * @brief       RS485��ѯ���յ�������
 * @param       buf     : ���ջ������׵�ַ
 * @param       len     : ���յ������ݳ���
 *   @arg               0   , ��ʾû�н��յ��κ�����
 *   @arg               ����, ��ʾ���յ������ݳ���
 * @retval      ��
 */
void rs485_receive_data(uint8_t *buf, uint8_t *len)
{
    uint8_t rxlen = g_RS485_rx_cnt;
    uint8_t i = 0;
    *len = 0;     /* Ĭ��Ϊ0 */
    delay_ms(10); /* �ȴ�10ms,��������10msû�н��յ�һ������,����Ϊ���ս��� */

    if (rxlen == g_RS485_rx_cnt && rxlen) /* ���յ�������,�ҽ�������� */
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_RS485_rx_buf[i];
        }

        *len = g_RS485_rx_cnt; /* ��¼�������ݳ��� */
        g_RS485_rx_cnt = 0;    /* ���� */
    }
}
