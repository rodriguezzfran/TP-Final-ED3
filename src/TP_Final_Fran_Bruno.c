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
void conf_DMA(uint8_t P2M);
void configUART(void);
void initPWMs(void);

void conf_PWM_Red();
void conf_PWM_Green();
void conf_PWM_Blue();

void turn_on_PWMs(void);
void turn_off_PWMs(void);
void processReceivedData(void);
void adapt_signal_data_for_dac(void);
void delay(uint32_t times);


/*
 * ADC_Freq = 120 [Hz] -> T between samples = 8.33[ms].
 * Then that must be the DAC timeout -> T_ticks = PCLK * T_out -> T_ticks = 25[MHz] * 8.33[ms] = 208333,33;
 */
#define ADC_FREQ 120
#define DAC_TOUT 208333

#define REDLED_LPC		(1<<22)
#define GREENLED_LPC 	(1<<25)
#define BLUELED_LPC 	(1<<26)

#define UART_RX_BUFFER_SIZE 4

uint32_t led_signal[360]; //360 samples
uint8_t adc_converting = 0;
uint8_t mode = 0; //0: RGB lights controlled by UART from PC. 1: ADC signal recreation
uint8_t uartRxBuffer[4] = "";

GPDMA_LLI_Type DMA_list;
int main(void) {


	conf_GPIO();
	conf_EXTI0();
	conf_EXTI1();
	conf_ADC();
	conf_DAC();
	configUART();
	//init_PWMs();//implement

	//Set P0.3 for adc signal
	GPIO_SetValue(0, 0x8);

	//UART_TxCmd(LPC_UART2, ENABLE);
	//UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
	GPIO_SetValue(3, GREENLED_LPC);
	NVIC_EnableIRQ(UART2_IRQn);

	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);

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

	pinc.Pinnum = PINSEL_PIN_3;
	PINSEL_ConfigPin(&pinc);

	//Set P0.0 P0.1 P0.2 and P0.3 as output
	GPIO_SetDir(0,0x15,1);


	//Also we configure the integrated led's for user alerts
	//P0.22 as GPIO for red led
	pinc.Pinnum = PINSEL_PIN_22;
	PINSEL_ConfigPin(&pinc);

	//P3.25 as GPIO for green led
	pinc.Portnum = PINSEL_PORT_3;
	pinc.Pinnum = PINSEL_PIN_25;
	PINSEL_ConfigPin(&pinc);

	//P3.26 as GPIO for blue led
	pinc.Pinnum = PINSEL_PIN_26;
	PINSEL_ConfigPin(&pinc);

	//Set P0.22 P3.25 and P3.26 as output
	GPIO_SetDir(0, REDLED_LPC, 1);
	GPIO_SetDir(3, (GREENLED_LPC | BLUELED_LPC), 1);

	//Shut down led's
	LPC_GPIO0->FIOSET |= REDLED_LPC;
	LPC_GPIO3->FIOSET |= (GREENLED_LPC | BLUELED_LPC);
}

//External interrupt configuration for general modes changing
void conf_EXTI0(void){
	NVIC_DisableIRQ(EINT0_IRQn);
	//P2.10 as EINT0
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_2;
	pinc.Pinnum = PINSEL_PIN_10;
	pinc.Pinmode = PINSEL_PINMODE_PULLUP;
	pinc.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&pinc);

	EXTI_SetMode(EXTI_EINT0,EXTI_MODE_EDGE_SENSITIVE);
	EXTI_SetPolarity(EXTI_EINT0,EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);


}

//External interrupt configuration for change mode between ADC reading and DAC output in second general mode
void conf_EXTI1(void){
	NVIC_DisableIRQ(EINT1_IRQn);
	//P2.11 as EINT1
	PINSEL_CFG_Type pinc;
	pinc.Portnum = PINSEL_PORT_2;
	pinc.Pinnum = PINSEL_PIN_11;
	pinc.Pinmode = PINSEL_PINMODE_PULLUP;
	pinc.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&pinc);

	EXTI_SetMode(EXTI_EINT1,EXTI_MODE_EDGE_SENSITIVE);
	EXTI_SetPolarity(EXTI_EINT1,EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);


}

void conf_ADC(void){
	NVIC_DisableIRQ(ADC_IRQn);
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
	DAC_UpdateValue(LPC_DAC, 0);
}

//DMA configuration function. 'P2M' is a flag to know if the transfer is ADC to Mem or Mem to DAC.
void conf_DMA(uint8_t P2M){
	NVIC_DisableIRQ(DMA_IRQn);
	if(!P2M){//If transfer type is M2P (DAC to memory), then configure LLI
		DMA_list.SrcAddr = (uint32_t)led_signal;
		DMA_list.DstAddr = (uint32_t)&(LPC_DAC->DACR);
		DMA_list.NextLLI = (uint32_t)&DMA_list;
		DMA_list.Control = (sizeof(led_signal) - 1)	//Transfer size
					  |(2<<18)	//Source width = 32 bits
					  |(2<<21);	//Destination width = 32 bits
		//Set SI and clear DI
		DMA_list.Control &= ~(1<<27);
		DMA_list.Control |= (1<<26);
	}

	GPDMA_Init();

	GPDMA_Channel_CFG_Type dmac;
	dmac.ChannelNum = 0;
	dmac.SrcMemAddr = (uint32_t)((P2M) ? 0 : led_signal);
	dmac.DstMemAddr = (uint32_t)((P2M) ? led_signal : 0);
	dmac.TransferSize = sizeof(led_signal) - 1;
	dmac.TransferWidth = 0;
	dmac.TransferType = (P2M) ? GPDMA_TRANSFERTYPE_P2M : GPDMA_TRANSFERTYPE_M2P;
	dmac.SrcConn = (P2M) ? GPDMA_CONN_ADC : 0;
	dmac.DstConn = (P2M) ? 0 : GPDMA_CONN_DAC;
	dmac.DMALLI = (uint32_t)((P2M) ? 0 : &DMA_list);
	GPDMA_Setup(&dmac);
}

void configUART(void) {
    // Configura pines para UART2
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 10;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    UART_CFG_Type      UARTConfigStruct;
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    //configuraci�n por defecto:
    UART_ConfigStructInit(&UARTConfigStruct);
    UARTConfigStruct.Baud_rate = 480600;
    //inicializa perif�rico
    UART_Init(LPC_UART2, &UARTConfigStruct);
    //Inicializa FIFO
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
    // Habilita interrupci�n por el RX del UART
    UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
    // Habilita interrupci�n por el estado de la linea UART
    UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, ENABLE);
    return;
}

void UART2_IRQHandler(void) {
	GPIO_ClearValue(3, GREENLED_LPC);
	uint32_t intsrc, tmp, tmp1;
		//Determina la fuente de interrupci�n
		intsrc = UART_GetIntId(LPC_UART2);
		tmp = intsrc & UART_IIR_INTID_MASK;
		// Eval�a Line Status
		if (tmp == UART_IIR_INTID_RLS){
			tmp1 = UART_GetLineStatus(LPC_UART2);
			tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
					| UART_LSR_BI | UART_LSR_RXFE);
			// ingresa a un Loop infinito si hay error
			if (tmp1) {
				while(1){};
			}
		}
		// Receive Data Available or Character time-out
		if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			UART_Receive(LPC_UART2, uartRxBuffer, sizeof(uartRxBuffer), BLOCKING);
		}
		GPIO_SetValue(3, GREENLED_LPC);
		return;
}

void processReceivedData(void){

}

void EINT0_IRQHandler(){
	if(mode == 0){
		//Disable UART
		// Deshabilita interrupci�n por el RX del UART
		UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, DISABLE);
		// Deshabilita interrupci�n por el estado de la linea UART
		UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, DISABLE);

		//Shutdown PWM signals on GPIO pins
		turn_off_PWMs();


		//Configure DMA and enable its channel and ADC channel
		conf_DMA(1);
		NVIC_EnableIRQ(DMA_IRQn);
		GPDMA_ChannelCmd(0, ENABLE);
		ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
		adc_converting = 1;
		mode = 1;
	}
	else{
		if(adc_converting){
			//Alert to user and do nothing
			LPC_GPIO0->FIOCLR |= REDLED_LPC;
			delay(2000);
			LPC_GPIO0->FIOSET |= REDLED_LPC;
		}
		else{
			//Stop DMA transfer and disable its channel cleanly
			LPC_GPDMACH0->DMACCControl &= ~(1<<18); //Disable requests for channel 0
			while(LPC_GPDMACH0->DMACCControl & (1<<17)); //Wait for possible data in channels FIFO
			GPDMA_ChannelCmd(0,DISABLE);

			DAC_UpdateValue(LPC_DAC, 0);

			//Enable UART and PWM signals again
			// Habilita interrupci�n por el RX del UART
			UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
			// Habilita interrupci�n por el estado de la linea UART
			UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, ENABLE);

			turn_on_PWMs();

			mode=0;
		}
	}

	EXTI_ClearEXTIFlag(EXTI_EINT0);
}

void EINT1_IRQHandler(){
	if(mode == 1){
		if(adc_converting){
			//Alert to user and do nothing
			LPC_GPIO0->FIOCLR |= REDLED_LPC;
			delay(2000);
			LPC_GPIO0->FIOSET |= REDLED_LPC;
		}
		else{
			LPC_GPDMACH0->DMACCControl &= ~(1<<18); //Disable requests for channel 0
			while(LPC_GPDMACH0->DMACCControl & (1<<17)); //Wait for possible data in channels FIFO
			GPDMA_ChannelCmd(0,DISABLE);
			conf_DMA(1);
			GPDMA_ChannelCmd(0, ENABLE);
			NVIC_EnableIRQ(DMA_IRQn);
			ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
			adc_converting = 1;
		}
	}
	else{
		//Alert to user and do nothing
		LPC_GPIO3->FIOCLR |= BLUELED_LPC;
		delay(2000);
		LPC_GPIO3->FIOSET |= BLUELED_LPC;
	}


	EXTI_ClearEXTIFlag(EXTI_EINT1);
}

void DMA_IRQHandler(){
	if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)){
		if(adc_converting){
			while(!ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE));//Wait for ADC to finish
			ADC_ChannelCmd(LPC_ADC, 0, DISABLE);

			LPC_GPDMACH0->DMACCControl &= ~(1<<18); //Disable requests for channel 0
			while(LPC_GPDMACH0->DMACCControl & (1<<17)); //Wait for possible data in channels FIFO
			GPDMA_ChannelCmd(0,DISABLE);

			adapt_signal_data_for_dac();
			conf_DMA(0);//Configure DMA for DAC to memory transfer
			GPDMA_ChannelCmd(0, ENABLE);
			adc_converting = 0;
			//NVIC_DisableIRQ(DMA_IRQn);//Disable DMA interrupts
		}
	}

	GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
	GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, 0);
}

void adapt_signal_data_for_dac(void){
	for(int i =0 ; i<sizeof(led_signal);i++){
		led_signal[i] &= (0x3FF << 6);
	}
}

void turn_off_PWMs(void){
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(TIMER1_IRQn);
	NVIC_DisableIRQ(TIMER2_IRQn);

	//TIM_Cmd(LPC_TIM0,DISABLE);
	//TIM_Cmd(LPC_TIM1,DISABLE);
	//TIM_Cmd(LPC_TIM2,DISABLE);
	GPIO_ClearValue(0, 0x7);
}


void turn_on_PWMs(void){
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	//TIM_Cmd(LPC_TIM0,ENABLE);
	//TIM_Cmd(LPC_TIM1,ENABLE);
	//TIM_Cmd(LPC_TIM2,ENABLE);
}

void delay(uint32_t times) {
	for(uint32_t i=0; i<times; i++)
		for(uint32_t j=0; j<times; j++);
}
