/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2016, MultiTech Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of MultiTech nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#include "mtqn_low_power.h"

#include "stdio.h"
#include "mbed_debug.h"

static uint32_t portA[6];
static uint32_t portB[6];
static uint32_t portC[6];
static uint32_t portD[6];
static uint32_t portE[6];
static uint32_t portF[6];
static uint32_t portG[6];
static uint32_t portH[6];

void mtqn_disable_systick_int() {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void mtqn_enable_systick_int() {
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

void mtqn_save_gpio_state() {
    portA[0] = GPIOA->MODER;
    portA[1] = GPIOA->OTYPER;
    portA[2] = GPIOA->OSPEEDR;
    portA[3] = GPIOA->PUPDR;
    portA[4] = GPIOA->AFR[0];
    portA[5] = GPIOA->AFR[1];

    portB[0] = GPIOB->MODER;
    portB[1] = GPIOB->OTYPER;
    portB[2] = GPIOB->OSPEEDR;
    portB[3] = GPIOB->PUPDR;
    portB[4] = GPIOB->AFR[0];
    portB[5] = GPIOB->AFR[1];

    portC[0] = GPIOC->MODER;
    portC[1] = GPIOC->OTYPER;
    portC[2] = GPIOC->OSPEEDR;
    portC[3] = GPIOC->PUPDR;
    portC[4] = GPIOC->AFR[0];
    portC[5] = GPIOC->AFR[1];

    portD[0] = GPIOD->MODER;
    portD[1] = GPIOD->OTYPER;
    portD[2] = GPIOD->OSPEEDR;
    portD[3] = GPIOD->PUPDR;
    portD[4] = GPIOD->AFR[0];
    portD[5] = GPIOD->AFR[1];

    portD[0] = GPIOD->MODER;
    portD[1] = GPIOD->OTYPER;
    portD[2] = GPIOD->OSPEEDR;
    portD[3] = GPIOD->PUPDR;
    portD[4] = GPIOD->AFR[0];
    portD[5] = GPIOD->AFR[1];

    portE[0] = GPIOE->MODER;
    portE[1] = GPIOE->OTYPER;
    portE[2] = GPIOE->OSPEEDR;
    portE[3] = GPIOE->PUPDR;
    portE[4] = GPIOE->AFR[0];
    portE[5] = GPIOE->AFR[1];

    portF[0] = GPIOF->MODER;
    portF[1] = GPIOF->OTYPER;
    portF[2] = GPIOF->OSPEEDR;
    portF[3] = GPIOF->PUPDR;
    portF[4] = GPIOF->AFR[0];
    portF[5] = GPIOF->AFR[1];

    portG[0] = GPIOG->MODER;
    portG[1] = GPIOG->OTYPER;
    portG[2] = GPIOG->OSPEEDR;
    portG[3] = GPIOG->PUPDR;
    portG[4] = GPIOG->AFR[0];
    portG[5] = GPIOG->AFR[1];

    portH[0] = GPIOH->MODER;
    portH[1] = GPIOH->OTYPER;
    portH[2] = GPIOH->OSPEEDR;
    portH[3] = GPIOH->PUPDR;
    portH[4] = GPIOH->AFR[0];
    portH[5] = GPIOH->AFR[1];
}

void mtqn_restore_gpio_state() {
    GPIOA->MODER = portA[0];
    GPIOA->OTYPER = portA[1];
    GPIOA->OSPEEDR = portA[2];
    GPIOA->PUPDR = portA[3];
    GPIOA->AFR[0] = portA[4];
    GPIOA->AFR[1] = portA[5];

    GPIOB->MODER = portB[0];
    GPIOB->OTYPER = portB[1];
    GPIOB->OSPEEDR = portB[2];
    GPIOB->PUPDR = portB[3];
    GPIOB->AFR[0] = portB[4];
    GPIOB->AFR[1] = portB[5];

    GPIOC->MODER = portC[0];
    GPIOC->OTYPER = portC[1];
    GPIOC->OSPEEDR = portC[2];
    GPIOC->PUPDR = portC[3];
    GPIOC->AFR[0] = portC[4];
    GPIOC->AFR[1] = portC[5];

    GPIOD->MODER = portD[0];
    GPIOD->OTYPER = portD[1];
    GPIOD->OSPEEDR = portD[2];
    GPIOD->PUPDR = portD[3];
    GPIOD->AFR[0] = portD[4];
    GPIOD->AFR[1] = portD[5];

    GPIOE->MODER = portE[0];
    GPIOE->OTYPER = portE[1];
    GPIOE->OSPEEDR = portE[2];
    GPIOE->PUPDR = portE[3];
    GPIOE->AFR[0] = portE[4];
    GPIOE->AFR[1] = portE[5];

    GPIOF->MODER = portF[0];
    GPIOF->OTYPER = portF[1];
    GPIOF->OSPEEDR = portF[2];
    GPIOF->PUPDR = portF[3];
    GPIOF->AFR[0] = portF[4];
    GPIOF->AFR[1] = portF[5];

    GPIOG->MODER = portG[0];
    GPIOG->OTYPER = portG[1];
    GPIOG->OSPEEDR = portG[2];
    GPIOG->PUPDR = portG[3];
    GPIOG->AFR[0] = portG[4];
    GPIOG->AFR[1] = portG[5];

    GPIOH->MODER = portH[0];
    GPIOH->OTYPER = portH[1];
    GPIOH->OSPEEDR = portH[2];
    GPIOH->PUPDR = portH[3];
    GPIOH->AFR[0] = portH[4];
    GPIOH->AFR[1] = portH[5];
}

/**
    * @brief  System Clock Speed decrease
    *         The system Clock source is shifted from HSI to MSI
    *         while at the same time, MSI range is set to RCC_MSIRANGE_0
    *         to go down to 100 KHz
    * @param  None
    * @retval None
    */
int SystemClock_Decrease(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* MSI is enabled in range 0 (100Khz) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    /* Initialization Error */
        return -1;
    }

    /* Select MSI as system clock source and keep HCLK, PCLK1 and PCLK2 clocks dividers as before */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        /* Initialization Error */
        return -1;
    }

    /* Disable HSI to reduce power consumption since MSI is used from that point */
    __HAL_RCC_HSI_DISABLE();
    __HAL_RCC_LSI_DISABLE();

    return 0;
}

// float internal pins not pins applications use. Leave those pins as the application needs them
void mtqn_float_internal_pins(){
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_PWREx_EnablePullUpPullDownConfig();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
    __HAL_RCC_GPIOG_CLK_DISABLE();

}

void mtqn_float_pins(){
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_PWREx_EnablePullUpPullDownConfig();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

/*
The ADCx (x=1,2,3), temperature sensor and VREFBUF buffer can consume power during
the Stop 0 mode, unless they are disabled before entering this mode.
Software procedure to disable the ADC
    1. Check that both ADSTART=0 and JADSTART=0 to ensure that no conversion is
    ongoing. If required, stop any regular and injected conversion ongoing by setting
    ADSTP=1 and JADSTP=1 and then wait until ADSTP=0 and JADSTP=0.
    2. Set ADDIS=1.
    3. If required by the application, wait until ADEN=0, until the analog ADC is effectively
    disabled (ADDIS will automatically be reset once ADEN=0).*/
void mtqn_disable_ADCx(){
    if(ADC1->CR & ADC_CR_ADEN){
        if (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)){
            ADC1->CR |= (ADC_CR_ADSTP | ADC_CR_JADSTP);
            while (ADC1->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP)){;}
            ADC1->CR |= ADC_CR_ADDIS;
            while (ADC1->CR & ADC_CR_ADEN){;}
        }
    }
    if(ADC2->CR & ADC_CR_ADEN){
        if (ADC2->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)){
            ADC2->CR |= (ADC_CR_ADSTP | ADC_CR_JADSTP);
            while (ADC2->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP)){;}
            ADC2->CR |= ADC_CR_ADDIS;
            while (ADC2->CR & ADC_CR_ADEN){;}
        }
    }
    if(ADC3->CR & ADC_CR_ADEN){
        if (ADC3->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)){
            ADC3->CR |= (ADC_CR_ADSTP | ADC_CR_JADSTP);
            while (ADC3->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP)){;}
            ADC3->CR |= ADC_CR_ADDIS;
            while (ADC3->CR & ADC_CR_ADEN){;}
        }
    }
}

void mtqn_disable_LPTIM1(){
    LPTIM1->CR &= ~LPTIM_CR_ENABLE;
}

void mtqn_disable_I2C3(){
    I2C3->CR1 &= ~I2C_CR1_PE;
}

void mtqn_disable_DMAx(){
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    DMA2_Channel1->CCR &= ~DMA_CCR_EN;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    DMA2_Channel3->CCR &= ~DMA_CCR_EN;
    DMA2_Channel4->CCR &= ~DMA_CCR_EN;
    DMA2_Channel5->CCR &= ~DMA_CCR_EN;
    DMA2_Channel6->CCR &= ~DMA_CCR_EN;
    DMA2_Channel7->CCR &= ~DMA_CCR_EN;
}

/*In order to go into low-power mode without generating errors on the line, the TE bit
must be reset before and the software must wait for the TC bit in the LPUART_ISR to
be set before resetting the UE bit.
The DMA requests are also reset when UE = 0 so the DMA channel must be disabled
before resetting the UE bit.*/
void mtqn_disable_LPUART(){
    //reset CR1_TE bit.
    LPUART1->CR1 &= ~USART_CR1_UE;
    //wait for TC bit in LPUART_ISR.
    volatile int tc;
    do{
        tc = LPUART1->ISR & USART_ISR_TC;
    } while(!tc);
    //reset CR1_UE bit.
    LPUART1->CR1 &= USART_CR1_UE;
}

void mtqn_disable_comparators(){
    COMP1->CSR &= ~COMP_CSR_EN;
    COMP2->CSR &= ~COMP_CSR_EN;
}

void mtqn_disable_PVMx(){
    PWR->CR2 &= ~PWR_CR2_PVME;
}

void mtqn_disable_PVD(){
    PWR->CR2 &= ~PWR_CR2_PVDE;
}

void mtqn_disable_OPAMPx(){
    OPAMP1->CSR &= ~OPAMP1_CSR_OPAEN;
    OPAMP2->CSR &= ~OPAMP2_CSR_OPAEN;
}

void mtqn_disable_DACx(){
    DAC->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2);
}

// Note: Software is allowed to write this bit only when the ADCs are disabled (ADCAL=0,
//   JADSTART=0, ADSTART=0, ADSTP=0, ADDIS=0 and ADEN=0)
void mtqn_disable_temp_sensor(){
    ADC123_COMMON->CCR &= ~ADC_CCR_TSEN;
}

void mtqn_disable_VREFBUF(){
    VREFBUF->CSR &= ~VREFBUF_CSR_ENVR;
}

void mtqn_enter_stop_mode2() {
    mtqn_disable_ADCx();
    mtqn_disable_LPTIM1();
    mtqn_disable_I2C3();
    mtqn_disable_DMAx();
    mtqn_disable_LPUART();
    mtqn_disable_comparators();
    mtqn_disable_PVMx();
    mtqn_disable_PVD();
    mtqn_disable_OPAMPx();
    mtqn_disable_DACx();
    mtqn_disable_temp_sensor();
    mtqn_disable_VREFBUF();

    mtqn_save_gpio_state();
    mtqn_float_pins();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

void mtqn_enter_stop_mode() {
    mtqn_disable_ADCx();
/*
Several peripherals can be used in Stop 0 mode and can add consumption if they are
enabled and clocked by LSI or LSE, or when they request the HSI16 clock: LPTIM1,

LPTIM2, I2Cx (x=1,2,3) U(S)ARTx(x=1,2...5), LPUART.
The DACx (x=1,2), the OPAMPs and the comparators can be used in Stop 0 mode, the
PVMx (x=1,2,3,4) and the PVD as well. If they are not needed, they must be disabled by
software to save their power consumptions.

The ADCx (x=1,2,3), temperature sensor and VREFBUF buffer can consume power during
the Stop 0 mode, unless they are disabled before entering this mode.
*/


    mtqn_save_gpio_state();
    mtqn_float_pins();

    SystemClock_Decrease();
    /* Suspend Tick increment for power consumption purposes         */
    HAL_SuspendTick();
    __HAL_RCC_TIM5_CLK_DISABLE();

    // make sure wakeup flag is cleared
    __HAL_PWR_CLEAR_FLAG(
    PWR_FLAG_WUF1 | PWR_FLAG_WUF2 | PWR_FLAG_WUF3 | PWR_FLAG_WUF4
    | PWR_FLAG_WUF5 | PWR_FLAG_WUFI);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    HAL_PWREx_EnableInternalWakeUpLine();
    HAL_PWREx_EnableLowPowerRunMode();


    /* Enable Power Clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Ensure that MSI is wake-up system clock */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    HAL_PWREx_DisableLowPowerRunMode();

    SetSysClock();
    SystemCoreClockUpdate();

    /* Resume Tick interrupt if disabled prior to Low Power Run mode entry */
    HAL_ResumeTick();
    __HAL_RCC_TIM5_CLK_ENABLE();

    mtqn_restore_gpio_state();
}

void mtqn_enter_standby_mode() {
    //mtqn_float_pins();
    //mtqn_pull_down_pins();

    /* Enable Power Clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Disable all used wakeup sources: WKUP pin */
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);


    /* Clear wake up Flag */
    __HAL_PWR_CLEAR_FLAG(
        PWR_FLAG_WUF1 | PWR_FLAG_WUF2 | PWR_FLAG_WUF3 | PWR_FLAG_WUF4
        | PWR_FLAG_WUF5 | PWR_FLAG_WUFI);

    /* Enable wakeup pin WKUP2 */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);

    /* Set RTC back-up register RTC_BKP31R to indicate
     later on that system has entered shutdown mode  */
     WRITE_REG( RTC->BKP31R, 0x1 );
     /* Enter shutdown mode */

     HAL_PWREx_EnterSHUTDOWNMode();
}

void mtqn_enable_standby_wake_pin() {
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
}

void mtqn_disable_standby_wake_pin() {
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
}
