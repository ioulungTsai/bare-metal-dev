/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 15 May 2024
 *      Author: Brandon Tsai
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescaler[4]  = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClock()
{
        return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
        uint32_t pclk1, SystemClk;

        uint8_t clksrc, temp, ahbp, abp2p;

        clksrc = ((RCC->CFGR >> 2) & 0x3);

        if (clksrc == 0) {
                SystemClk = 16000000;
        } else if (clksrc == 1) {
                SystemClk = 8000000;
        } else if (clksrc == 2) {
                SystemClk = RCC_GetPLLOutputClock();
        }

        // AHB
        temp = ((RCC->CFGR >> 4) & 0xF);

        if (temp < 8) {
                ahbp = 1;
        } else {
                ahbp = AHB_Prescaler[temp - 8];
        }

        // APB1
        temp = ((RCC->CFGR >> 10) & 0x7);

        if (temp < 4) {
                abp2p = 1;
        } else {
                abp2p = APB_Prescaler[temp - 4];
        }

        pclk1 = (SystemClk / ahbp) / abp2p;

        return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
        uint32_t pclk2, SystemClk;

        uint8_t clksrc, temp, ahbp, abp2p;

        clksrc = ((RCC->CFGR >> 2) & 0x3);

        if (clksrc == 0) {
                SystemClk = 16000000;
        } else if (clksrc == 1) {
                SystemClk = 8000000;
        } else if (clksrc == 2) {
                SystemClk = RCC_GetPLLOutputClock();
        }

        // AHB
        temp = ((RCC->CFGR >> 4) & 0xF);

        if (temp < 8) {
                ahbp = 1;
        } else {
                ahbp = AHB_Prescaler[temp - 8];
        }

        // APB1
        temp = ((RCC->CFGR >> 13) & 0x7);

        if (temp < 4) {
                abp2p = 1;
        } else {
                abp2p = APB_Prescaler[temp - 4];
        }

        pclk2 = (SystemClk / ahbp) / abp2p;

        return pclk2;
}
