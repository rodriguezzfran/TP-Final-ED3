#include "LPC17xx.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

void conf_GPIO(void);
void conf_EXTI0(void);
void conf_EXTI1(void);
void conf_ADC(void);
void conf_DAC(void);
void conf_DMA(uint32_t *SrcAddr, uint32_t *DstAddr, uint8_t P2M);

void conf_PWM_Red();
void conf_PWM_Green();
void conf_PWM_Blue();

/*
 * ADC_Freq = 100 [Hz] -> T between samples = 10[ms].
 * Then that must be the DAC timeout -> T_ticks = PCLK * T_out -> T_ticks = 25[MHz] * 10[ms] = 250000;
 */
#define ADC_FREQ 100
#define DAC_TOUT 250000

uint32_t signal[300]; //300 samples (1,2KB)
int main(void) {



    while(1) {
    }
    return 0 ;
}


void conf_GPIO(void){
	PINSEL_CFG_Type pinc;

	//P0.0 as GPIO for RED led's (PWM)
	pinc.Portnum = PINSEL_PORT_0;
	pinc.Pinnum = PINSEL_PIN_0;
	pinc.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinc.Funcnum = PINSEL_FUNC_0;
	PINSEL_ConfigPin(&pinc);

	//P0.1 as GPIO for GREEN led's (PWM)
	pinc.Pinnum = PINSEL_PIN_1;
	PINSEL_ConfigPin(&pinc);

	//P0.2 as GPIO for BLUE led's (PWM)
	pinc.Pinnum = PINSEL_PIN_2;
	PINSEL_ConfigPin(&pinc);

	//Set P0.0 P0.1 and P0.2 as output
	GPIO_SetDir(0,0x7,1);
}

//External interrupt configuration for general modes changing
void conf_EXTI0(void){
	//P2.10 as EINT0
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_2;
	pinc.Pinnum = PINSEL_PIN_10;
	pinc.Pinmode = PINSEL_PINMODE_PULLUP;
	pinc.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&pinc);

	EXTI_SetMode(EXTI_EINT0,EXTI_MODE_EDGE_SENSITIVE);
	EXTI_SetPolarity(EXTI_EINT0,EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);

	NVIC_EnableIRQ(EINT0_IRQn);
}

//External interrupt configuration for change mode between ADC reading and DAC output in second general mode
void conf_EXTI1(void){
	//P2.11 as EINT1
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_2;
	pinc.Pinnum = PINSEL_PIN_11;
	pinc.Pinmode = PINSEL_PINMODE_PULLUP;
	pinc.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&pinc);

	EXTI_SetMode(EXTI_EINT1,EXTI_MODE_EDGE_SENSITIVE);
	EXTI_SetPolarity(EXTI_EINT1,EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);

	NVIC_EnableIRQ(EINT1_IRQn);
}

void conf_ADC(void){
	//P0.23 as AD0.0
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_0;
	pinc.Pinnum = PINSEL_PIN_23;
	pinc.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinc.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&pinc);

	ADC_Init(LPC_ADC,ADC_FREQ);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,ENABLE);//Enable interrupt for DMA request
	//Since this is a previous configuration, the channel will be disabled until we need the ADC (exti0 handler).
	ADC_BurstCmd(LPC_ADC,1);//Burst mode for DMA
	ADC_StartCmd(LPC_ADC,ADC_START_CONTINUOUS);
}

void conf_DAC(void){
	//P0.26 as AOUT
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_0;
	pinc.Pinnum = PINSEL_PIN_26;
	pinc.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinc.Funcnum = PINSEL_FUNC_2;
	PINSEL_ConfigPin(&pinc);

	DAC_CONVERTER_CFG_Type dacc;
	dacc.CNT_ENA = 1;
	dacc.DMA_ENA = 1;

	DAC_Init(LPC_DAC);
	DAC_ConfigDAConverterControl(LPC_DAC, &dacc);
	DAC_SetDMATimeOut(LPC_DAC, DAC_TOUT);
}

void conf_DMA(uint32_t *SrcAddr, uint32_t *DstAddr, uint8_t P2M){
	GPDMA_LLI_Type list;
	list.SrcAddr = SrcAddr;
	list.DstAddr = DstAddr;
	list.NextLLI = &list;
	list.Control = (2^13) | (2<<18) | (2<<21);
	//If the transfer type is P2M set DI and clear SI, else if it's M2P set SI and clear DI
	if(P2M){
		list.Control &= ~(1<<26);
		list.Control |= (1<<27);
	}
	else{
		list.Control &= ~(1<<27);
		list.Control |= (1<<26);
	}

	GPDMA_Init();

	GPDMA_Channel_CFG_Type dmac;
	dmac.ChannelNum = 0;
	dmac.SrcMemAddr = (P2M) ? 0 : SrcAddr;
	dmac.DstMemAddr = (P2M) ? DstAddr : 0;

}



