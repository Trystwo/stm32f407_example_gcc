#include "main.h"

void sram_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FSMC_NORSRAM_TimingTypeDef sram_timing;

    __HAL_RCC_FSMC_CLK_ENABLE();


}