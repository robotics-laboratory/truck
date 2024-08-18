#include <cstdint>
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"

#include "lpuart1.h"

#define __IO volatile

extern "C" {
void vApplicationTickHook() {
    LL_TIM_ClearFlag_UPDATE(TIM6);
}
}

void rtos_tick_timer_init() {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    /* TIM6 interrupt Init */
    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    TIM_InitStruct.Prescaler = 144;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 200;
    LL_TIM_Init(TIM6, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode(TIM6);
}

void board_start_rtos_timer() {
    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
}

static void system_clock_init() {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {};
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while(LL_RCC_HSE_IsReady() != 1) {};

    LL_RCC_HSI_Enable();
    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1) {};

    LL_RCC_HSI_SetCalibTrimming(64);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 24, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 24, LL_RCC_PLLQ_DIV_6);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_EnableDomain_48M();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1) {};

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {};

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (170 >> 1); i !=0; i--);

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(144000000);

    /* Update the time base */
    HAL_InitTick (TICK_INT_PRIORITY);
}

void board_init() {
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    /* HAL_RCC_SYSCFG_CLK_ENABLE */
    do {
        __IO uint32_t tmpreg;
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
        UNUSED(tmpreg);
    } while(0);

    /* __HAL_RCC_PWR_CLK_ENABLE */
    do {
        __IO uint32_t tmpreg;
        SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
        UNUSED(tmpreg);
   } while(0);

    /* Write 1 to disable the USB Type-C dead battery pull-down behavior */
    SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

    system_clock_init();
    rtos_tick_timer_init();
    lpuart1_init();
}