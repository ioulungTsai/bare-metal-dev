#include <stdint.h>

#define SRAM_START         0x20000000U
#define SRAM_SIZE          (128 * 1024) //128KB
#define SRAM_END           ((SRAM_START) + (SRAM_SIZE))

#define STACK_START        SRAM_END

#define WEAK_ALIAS_DEFAULT __attribute__((weak, alias("Default_Handler")))

extern uint32_t _etext;

extern uint32_t _la_data;

extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

void __libc_init_array(void);

void Reset_Handler(void);

void NMI_Handler(void) WEAK_ALIAS_DEFAULT;
void HardFault_Handler(void) WEAK_ALIAS_DEFAULT;
void MemManage_Handler(void) WEAK_ALIAS_DEFAULT;
void BusFault_Handler(void) WEAK_ALIAS_DEFAULT;
void UsageFault_Handler(void) WEAK_ALIAS_DEFAULT;
void SVC_Handler(void) WEAK_ALIAS_DEFAULT;
void DebugMon_Handler(void) WEAK_ALIAS_DEFAULT;
void PendSV_Handler(void) WEAK_ALIAS_DEFAULT;
void SysTick_Handler(void) WEAK_ALIAS_DEFAULT;
void WWDG_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void PVD_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TAMP_STAMP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void RTC_WKUP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void RCC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI0_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI4_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream0_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream4_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream5_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream6_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void ADC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN1_TX_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN1_RX0_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN1_RX1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN1_SCE_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI9_5_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM1_BRK_TIM9_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM1_UP_TIM10_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM1_TRG_COM_TIM11_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM1_CC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM4_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C1_EV_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C1_ER_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C2_EV_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C2_ER_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void SPI1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void SPI2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void USART1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void USART2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void USART3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void EXTI15_10_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void RTC_Alarm_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_FS_WKUP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM8_BRK_TIM12_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM8_UP_TIM13_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM8_TRG_COM_TIM14_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM8_CC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA1_Stream7_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void FSMC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void SDIO_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM5_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void SPI3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void UART4_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void UART5_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM6_DAC_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void TIM7_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream0_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream2_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream3_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream4_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void ETH_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void ETH_WKUP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN2_TX_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN2_RX0_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN2_RX1_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CAN2_SCE_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_FS_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream5_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream6_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DMA2_Stream7_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void USART6_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C3_EV_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void I2C3_ER_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_HS_EP1_OUT_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_HS_EP1_IN_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_HS_WKUP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void OTG_HS_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void DCMI_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void CRYP_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void HASH_RNG_IRQHandler(void) WEAK_ALIAS_DEFAULT;
void FPU_IRQHandler(void) WEAK_ALIAS_DEFAULT;

uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
        STACK_START,
        (uint32_t)Reset_Handler,
        (uint32_t)NMI_Handler,
        (uint32_t)HardFault_Handler,
        (uint32_t)MemManage_Handler,
        (uint32_t)BusFault_Handler,
        (uint32_t)UsageFault_Handler,
        0,
        0,
        0,
        0,
        (uint32_t)SVC_Handler,
        (uint32_t)DebugMon_Handler,
        0,
        (uint32_t)PendSV_Handler,
        (uint32_t)SysTick_Handler,
        (uint32_t)WWDG_IRQHandler,
        (uint32_t)PVD_IRQHandler,
        (uint32_t)TAMP_STAMP_IRQHandler,
        (uint32_t)RTC_WKUP_IRQHandler,
        0,
        (uint32_t)RCC_IRQHandler,
        (uint32_t)EXTI0_IRQHandler,
        (uint32_t)EXTI1_IRQHandler,
        (uint32_t)EXTI2_IRQHandler,
        (uint32_t)EXTI3_IRQHandler,
        (uint32_t)EXTI4_IRQHandler,
        (uint32_t)DMA1_Stream0_IRQHandler,
        (uint32_t)DMA1_Stream1_IRQHandler,
        (uint32_t)DMA1_Stream2_IRQHandler,
        (uint32_t)DMA1_Stream3_IRQHandler,
        (uint32_t)DMA1_Stream4_IRQHandler,
        (uint32_t)DMA1_Stream5_IRQHandler,
        (uint32_t)DMA1_Stream6_IRQHandler,
        (uint32_t)ADC_IRQHandler,
        (uint32_t)CAN1_TX_IRQHandler,
        (uint32_t)CAN1_RX0_IRQHandler,
        (uint32_t)CAN1_RX1_IRQHandler,
        (uint32_t)CAN1_SCE_IRQHandler,
        (uint32_t)EXTI9_5_IRQHandler,
        (uint32_t)TIM1_BRK_TIM9_IRQHandler,
        (uint32_t)TIM1_UP_TIM10_IRQHandler,
        (uint32_t)TIM1_TRG_COM_TIM11_IRQHandler,
        (uint32_t)TIM1_CC_IRQHandler,
        (uint32_t)TIM2_IRQHandler,
        (uint32_t)TIM3_IRQHandler,
        (uint32_t)TIM4_IRQHandler,
        (uint32_t)I2C1_EV_IRQHandler,
        (uint32_t)I2C1_ER_IRQHandler,
        (uint32_t)I2C2_EV_IRQHandler,
        (uint32_t)I2C2_ER_IRQHandler,
        (uint32_t)SPI1_IRQHandler,
        (uint32_t)SPI2_IRQHandler,
        (uint32_t)USART1_IRQHandler,
        (uint32_t)USART2_IRQHandler,
        (uint32_t)USART3_IRQHandler,
        (uint32_t)EXTI15_10_IRQHandler,
        (uint32_t)RTC_Alarm_IRQHandler,
        (uint32_t)OTG_FS_WKUP_IRQHandler,
        (uint32_t)TIM8_BRK_TIM12_IRQHandler,
        (uint32_t)TIM8_UP_TIM13_IRQHandler,
        (uint32_t)TIM8_TRG_COM_TIM14_IRQHandler,
        (uint32_t)TIM8_CC_IRQHandler,
        (uint32_t)DMA1_Stream7_IRQHandler,
        (uint32_t)FSMC_IRQHandler,
        (uint32_t)SDIO_IRQHandler,
        (uint32_t)TIM5_IRQHandler,
        (uint32_t)SPI3_IRQHandler,
        (uint32_t)UART4_IRQHandler,
        (uint32_t)UART5_IRQHandler,
        (uint32_t)TIM6_DAC_IRQHandler,
        (uint32_t)TIM7_IRQHandler,
        (uint32_t)DMA2_Stream0_IRQHandler,
        (uint32_t)DMA2_Stream1_IRQHandler,
        (uint32_t)DMA2_Stream2_IRQHandler,
        (uint32_t)DMA2_Stream3_IRQHandler,
        (uint32_t)DMA2_Stream4_IRQHandler,
        (uint32_t)ETH_IRQHandler,
        (uint32_t)ETH_WKUP_IRQHandler,
        (uint32_t)CAN2_TX_IRQHandler,
        (uint32_t)CAN2_RX0_IRQHandler,
        (uint32_t)CAN2_RX1_IRQHandler,
        (uint32_t)CAN2_SCE_IRQHandler,
        (uint32_t)OTG_FS_IRQHandler,
        (uint32_t)DMA2_Stream5_IRQHandler,
        (uint32_t)DMA2_Stream6_IRQHandler,
        (uint32_t)DMA2_Stream7_IRQHandler,
        (uint32_t)USART6_IRQHandler,
        (uint32_t)I2C3_EV_IRQHandler,
        (uint32_t)I2C3_ER_IRQHandler,
        (uint32_t)OTG_HS_EP1_OUT_IRQHandler,
        (uint32_t)OTG_HS_EP1_IN_IRQHandler,
        (uint32_t)OTG_HS_WKUP_IRQHandler,
        (uint32_t)OTG_HS_IRQHandler,
        (uint32_t)DCMI_IRQHandler,
        (uint32_t)CRYP_IRQHandler,
        (uint32_t)HASH_RNG_IRQHandler,
        (uint32_t)FPU_IRQHandler,
};

void Default_Handler(void)
{
        while (1)
                ;
}

void Reset_Handler(void)
{
        // copy .data section to SRAM
        uint32_t size             = (uint32_t)&_edata - (uint32_t)&_sdata;
        uint8_t *source_data      = (uint8_t *)&_la_data;
        uint8_t *destination_data = (uint8_t *)&_sdata;

        for (uint32_t i = 0; i < size; i++) {
                destination_data[i] = source_data[i];
        }

        // initial the .bss section to zero in SRAM
        size             = (uint32_t)&_ebss - (uint32_t)&_sbss;
        destination_data = (uint8_t *)&_sbss;

        for (uint32_t i = 0; i < size; i++) {
                destination_data[i] = 0;
        }

        // call initial function of standard library
        __libc_init_array();

        // call main()
        main();
}
