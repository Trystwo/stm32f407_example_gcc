/**
 * @file usart.c
 * @author Trstwo (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "main.h"
#include "usart.h"

/*arm-none-eabi-gcc编译器 printf重定向到串口1*/
int __io_putchar(int ch)
{
    while((USART1->SR & 0X40) == 0); // 循环发送,直到发送完毕
    USART1->DR = (uint8_t) ch;
    return ch;
}

int _write(int file, char *ptr, int len)
{
        int DataIdx;
        for (DataIdx = 0; DataIdx < len; DataIdx++)
        {
                __io_putchar( *ptr++ );
        }
        return len;
}

UART_HandleTypeDef uartx_handle;
uint8_t rx_buffer[1];

void usart_init(uint32_t baudrate)
{
    uartx_handle.Instance = USART1;
    uartx_handle.Init.BaudRate = baudrate;
    uartx_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uartx_handle.Init.StopBits = UART_STOPBITS_1;
    uartx_handle.Init.Parity = UART_PARITY_NONE; // No parity check
    uartx_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No flow control
    uartx_handle.Init.Mode = UART_MODE_TX_RX; // Enable both TX and RX
    
    if (HAL_UART_Init(&uartx_handle) != HAL_OK)
    {
        Error_Handler();
    }
    /* 使能串口1和串口1的中断，并设置接受缓冲区大小为1 */
    HAL_UART_Receive_IT(&uartx_handle, (uint8_t *)rx_buffer, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    gpio_init_struct.Pin = GPIO_PIN_9;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART1;
    
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    gpio_init_struct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
}