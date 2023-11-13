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
void conf_DMA(void);
void configUART(void);
void conf_Timer0(void);
void conf_Timer1(void);
void conf_Timer2(void);
void conf_Timer3(void);

void process_led_mode(uint8_t info);
void conf_PWM_Red(uint8_t redVal);
void conf_PWM_Green(uint8_t greenVal);
void conf_PWM_Blue(uint8_t blueVal);
void set_mode_normal(uint8_t redVal, uint8_t greenVal, uint8_t blueVal);
void set_mode_flash(void);
void set_mode_strobe(void);
void set_mode_fade(void);


void turn_on_PWMs(void);
void turn_off_PWMs(void);
void processReceivedData(void);
void delay(uint32_t times);


/*
 * ADC_Freq = 1502 [Hz] -> T between samples = 666[us].
 * Then that must be the DAC timeout -> T_ticks = PCLK * T_out -> T_ticks = 25[MHz] * 666[us] = 16644.47;
 */
#define ADC_FREQ 200000
#define SAMPLES_FREQ 500
//#define CANT_SAMPLES (5*SAMPLES_FREQ)
#define CANT_SAMPLES (30)
#define DAC_TOUT ((1/SAMPLES_FREQ)*25000000)

#define REDLED_LPC		(1<<22)
#define GREENLED_LPC 	(1<<25)
#define BLUELED_LPC 	(1<<26)

#define UART_RX_BUFFER_SIZE 4

#define LED_MODE_NORMAL 0
#define LED_MODE_FLASH 1
#define LED_MODE_STROBE 2
#define LED_MODE_FADE 3

#define LED_SPEED_10 1
#define LED_SPEED_30 3
#define LED_SPEED_50 5
#define LED_SPEED_70 7
#define LED_SPEED_90 9

const uint8_t LED_Mode[] = {
	LED_MODE_NORMAL,
	LED_MODE_FLASH,
	LED_MODE_STROBE,
	LED_MODE_FADE
};
const uint8_t LED_Speed[] = {
	LED_SPEED_10,
	LED_SPEED_30,
	LED_SPEED_50,
	LED_SPEED_70,
	LED_SPEED_90
};

uint32_t led_signal[CANT_SAMPLES]; //In a range of 5 seconds, 2500 data will be loaded with a rate of 1000[Hz] (1 data in 1[ms])
uint32_t auxiliar[CANT_SAMPLES];
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE] = "";

volatile uint8_t uartflag = 0;
volatile uint8_t adc_converting = 0;
volatile uint8_t mode = 0; //0: RGB lights controlled by UART from PC. 1: ADC signal recreation.
volatile uint16_t data_counter = 0;
volatile uint16_t one_sec_check = 0;
volatile uint8_t adc_preparing = 0;
volatile uint8_t strobe_mode_slope = 1; //0: strobe mode getting brighter. 1: strobe mode becoming less bright.
volatile uint8_t fade_mode_state = 0;

//Match values for PWMs duty cycles
volatile uint8_t current_red_pwm_duty = 50 - 1;
volatile uint8_t current_green_pwm_duty = 50 - 1;
volatile uint8_t current_blue_pwm_duty = 50 - 1;

volatile uint8_t led_mode = 0;
volatile uint8_t led_speed = 0;

GPDMA_LLI_Type DMA_list;

int main(void) {
	for(int i = 0; i < CANT_SAMPLES ; i++){
		auxiliar[i]=(i*30);
	}

	//Clear Timers, EINT0/1, UART2, ADC and DMA pending interrupts
	NVIC->ICPR[0] |= 0b011110010000000001100100010000;

	conf_GPIO();

	conf_EXTI0();
	conf_EXTI1();
	conf_ADC();
	conf_DAC();
	configUART();
	conf_DMA();

	conf_Timer0();//Timer for red led PWM
	conf_Timer1();//Timer for green led PWM
	conf_Timer2();//Timer for blue led PWM
	conf_Timer3();//Timer for different modes

	//Set P0.3 for ADC signal
	GPIO_SetValue(0, 0x8);

	GPIO_SetValue(3, GREENLED_LPC);

	NVIC_EnableIRQ(UART2_IRQn);
	//NVIC_EnableIRQ(EINT0_IRQn);
	//NVIC_EnableIRQ(EINT1_IRQn);
	turn_on_PWMs();

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
	GPIO_SetDir(0,0b1111,1);

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

void conf_Timer0(void){
	TIM_TIMERCFG_Type timc;
	timc.PrescaleOption=TIM_PRESCALE_TICKVAL;
	timc.PrescaleValue = 2500; //Tr = 100[us]

	TIM_MATCHCFG_Type matc;
	matc.MatchChannel = 0;
	matc.IntOnMatch = ENABLE;
	matc.ResetOnMatch = ENABLE;
	matc.StopOnMatch = DISABLE;
	matc.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	matc.MatchValue = (uint32_t)(current_red_pwm_duty);

	TIM_Init(LPC_TIM0,TIM_TIMER_MODE,&timc);
	TIM_ConfigMatch(LPC_TIM0, &matc);

	NVIC_SetPriority(TIMER2_IRQn, 5);
}

void conf_Timer1(void){
	TIM_TIMERCFG_Type timc;
	timc.PrescaleOption=TIM_PRESCALE_TICKVAL;
	timc.PrescaleValue = 2500; //Tr = 100[us]

	TIM_MATCHCFG_Type matc;
	matc.MatchChannel = 0;
	matc.IntOnMatch = ENABLE;
	matc.ResetOnMatch = ENABLE;
	matc.StopOnMatch = DISABLE;
	matc.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	matc.MatchValue = (uint32_t)(current_green_pwm_duty);

	TIM_Init(LPC_TIM1,TIM_TIMER_MODE,&timc);
	TIM_ConfigMatch(LPC_TIM1, &matc);

	NVIC_SetPriority(TIMER2_IRQn, 5);
}

void conf_Timer2(void){
	TIM_TIMERCFG_Type timc;
	timc.PrescaleOption=TIM_PRESCALE_TICKVAL;
	timc.PrescaleValue = 2500; //Tr = 100[us]

	TIM_MATCHCFG_Type matc;
	matc.MatchChannel = 0;
	matc.IntOnMatch = ENABLE;
	matc.ResetOnMatch = ENABLE;
	matc.StopOnMatch = DISABLE;
	matc.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	matc.MatchValue = (uint32_t)(current_blue_pwm_duty);

	TIM_Init(LPC_TIM2,TIM_TIMER_MODE,&timc);
	TIM_ConfigMatch(LPC_TIM2, &matc);

	NVIC_SetPriority(TIMER2_IRQn, 5);
}

void conf_Timer3(void){
	NVIC_DisableIRQ(TIMER3_IRQn);
	//TIM_Cmd(LPC_TIM3, DISABLE);
	TIM_TIMERCFG_Type timc;
	timc.PrescaleOption=TIM_PRESCALE_TICKVAL;
	timc.PrescaleValue = 2500; //Tr = 100[us]

	TIM_MATCHCFG_Type matc;
	matc.MatchChannel = 0;
	matc.IntOnMatch = ENABLE;
	matc.ResetOnMatch = ENABLE;
	matc.StopOnMatch = DISABLE;
	matc.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	matc.MatchValue = (uint32_t)(20); //This match value doesn't matter for now because this timer will be used in other mode

	TIM_Init(LPC_TIM3,TIM_TIMER_MODE,&timc);
	TIM_ConfigMatch(LPC_TIM3, &matc);

	NVIC_SetPriority(TIMER2_IRQn, 5);
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
	NVIC_SetPriority(ADC_IRQn, 5);
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
	dacc.CNT_ENA = SET;
	dacc.DMA_ENA = SET;

	DAC_Init(LPC_DAC);
	DAC_ConfigDAConverterControl(LPC_DAC, &dacc);
	DAC_SetDMATimeOut(LPC_DAC, 12500000);
	DAC_UpdateValue(LPC_DAC, 0);
}

void conf_DMA(void){
	NVIC_DisableIRQ(DMA_IRQn);
	//GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 0);
	//GPDMA_ClearIntPending(GPDMA_STAT_INTERR, 0);

	DMA_list.SrcAddr = (uint32_t)auxiliar;
	DMA_list.DstAddr = (uint32_t)(&LPC_DAC->DACR);
	DMA_list.NextLLI = (uint32_t)(&DMA_list);
	DMA_list.Control = (CANT_SAMPLES)	//Transfer size
				  |(2<<18)	//Source width = 32 bits
				  |(2<<21)	//Destination width = 32 bits
				  |(1<<26);	//Set SI
	DMA_list.Control &= ~(1<<27); //Clear DI

	GPDMA_Init();

	GPDMA_Channel_CFG_Type dmac;
	dmac.ChannelNum = 0;
	dmac.SrcMemAddr = (uint32_t)auxiliar;
	dmac.DstMemAddr = 0;
	dmac.TransferSize = (CANT_SAMPLES);
	dmac.TransferWidth = 0;
	dmac.TransferType = GPDMA_TRANSFERTYPE_M2P;
	dmac.SrcConn = 0;
	dmac.DstConn = GPDMA_CONN_DAC;
	dmac.DMALLI = (uint32_t)(&DMA_list);
	GPDMA_Setup(&dmac);
	//LPC_GPDMACH0->DMACCControl |= (2<<18) | (2<<21);

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

    NVIC_SetPriority(UART2_IRQn, 3);
    return;
}

void UART2_IRQHandler(void) {
	uartflag++;
	if(uartflag>1){
		uartflag = 0;
	}
	else{
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
			UART_Receive(LPC_UART2, uartRxBuffer, UART_RX_BUFFER_SIZE, BLOCKING);
		}
		TIM_Cmd(LPC_TIM3, DISABLE);
		NVIC_DisableIRQ(TIMER3_IRQn);
		NVIC_ClearPendingIRQ(TIMER3_IRQn);
		processReceivedData();
	}
	return;
}

void EINT0_IRQHandler(void){
	if(mode == 0){
		//Disable UART
		// Deshabilita interrupci�n por el RX del UART
		UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, DISABLE);
		// Deshabilita interrupci�n por el estado de la linea UART
		UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, DISABLE);

		//Shutdown PWM signals on GPIO pins
		turn_off_PWMs();

		//Enable SysTick
		//GPIO_SetValue(3, GREENLED_LPC);
		if (SysTick_Config(SystemCoreClock/SAMPLES_FREQ)){	// Systick 1ms
			while (1); // En caso de error
		}
		NVIC_SetPriority(SysTick_IRQn, 5);

		adc_preparing = 1;
		mode = 1;
	}
	else{
		if(adc_converting || adc_preparing){
			//Alert to user and do nothing
			LPC_GPIO0->FIOCLR |= REDLED_LPC;
			delay(2000);
			LPC_GPIO0->FIOSET |= REDLED_LPC;
		}
		else{
			//Stop DMA transfer and disable its channel cleanly
			//LPC_GPDMACH0->DMACCConfig |= (1<<18); //Disable requests for channel 0
			//while(LPC_GPDMACH0->DMACCConfig & (1<<17)); //Wait for possible data in channels FIFO
			//GPDMA_ChannelCmd(0,DISABLE);
			LPC_GPDMACH0->DMACCConfig &= ~(0x1);

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
	return;
}

void EINT1_IRQHandler(void){
	if(mode == 1){
		if(adc_converting || adc_preparing){
			//Alert to user and do nothing
			LPC_GPIO0->FIOCLR |= REDLED_LPC;
			delay(2000);
			LPC_GPIO0->FIOSET |= REDLED_LPC;
		}
		else{
			//LPC_GPDMACH0->DMACCConfig |= (1<<18); //Disable requests for channel 0
			//while(LPC_GPDMACH0->DMACCConfig & (1<<17)); //Wait for possible data in channels FIFO
			//GPDMA_ChannelCmd(0,DISABLE);
			LPC_GPDMACH0->DMACCConfig &= ~(0x1);
			DAC_UpdateValue(LPC_DAC, 0);

			//GPIO_ClearValue(3,	GREENLED_LPC);
			adc_preparing = 1;
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

void ADC_IRQHandler(void){
	//ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
	if(data_counter < CANT_SAMPLES){
		//led_signal[data_counter] = LPC_ADC->ADDR0;
		led_signal[data_counter] = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
		data_counter++;
	}
	else{
		data_counter = 0;
		NVIC_DisableIRQ(ADC_IRQn);
		GPIO_SetValue(3, GREENLED_LPC);
		adc_converting = 0;

		//GPDMA_ChannelCmd(0, ENABLE);
		LPC_GPDMACH0->DMACCConfig |= (0x1);
	}

	LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
	LPC_ADC->ADDR0;
}

void SysTick_Handler(void){
	if(adc_preparing || adc_converting){
		if(adc_preparing){
			if(one_sec_check<(SAMPLES_FREQ*2)){//Count 2 seconds
				one_sec_check++;
			}
			else{
				adc_preparing = 0;
				one_sec_check = 0;
				adc_converting = 1;
				NVIC_EnableIRQ(ADC_IRQn);
				ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
				ADC_StartCmd(LPC_ADC, ADC_START_NOW);
				GPIO_ClearValue(3, GREENLED_LPC);
			}
		}
		else{
			//ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
			ADC_StartCmd(LPC_ADC, ADC_START_NOW);
		}
	}

	SysTick->CTRL &= SysTick->CTRL;//Clear flag
}

void TIMER0_IRQHandler(void){
	//TIM_Cmd(LPC_TIM0,DISABLE);
	LPC_TIM0->TCR |= 2;
	if(!(LPC_GPIO0->FIOPIN & (REDLED_LPC))){
		if(current_red_pwm_duty < 99){
			LPC_GPIO0->FIOSET |= REDLED_LPC;
			LPC_TIM0->MR0 = 99 - current_red_pwm_duty - 1;
		}
	}
	else{
		if(current_red_pwm_duty > 0){
			LPC_GPIO0->FIOCLR |= REDLED_LPC;
			LPC_TIM0->MR0 = (uint32_t)(current_red_pwm_duty);
		}
	}
	//TIM_Cmd(LPC_TIM0,ENABLE);
    LPC_TIM0->TCR &= ~(2);

	TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);
}

void TIMER1_IRQHandler(void){
	//TIM_Cmd(LPC_TIM0,DISABLE);
	 LPC_TIM1->TCR |= 2;
	if(!(LPC_GPIO3->FIOPIN & GREENLED_LPC)){
		if(current_green_pwm_duty < 99){
			LPC_GPIO3->FIOSET |= GREENLED_LPC;
			LPC_TIM1->MR0 = 99 - current_green_pwm_duty - 1;
		}
	}
	else{
		if(current_green_pwm_duty > 0){
			LPC_GPIO3->FIOCLR |= GREENLED_LPC;
			LPC_TIM1->MR0 = (uint32_t)(current_green_pwm_duty);
		}
	}
	//TIM_Cmd(LPC_TIM0,ENABLE);
    LPC_TIM1->TCR &= ~(2);

	TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);
}

void TIMER2_IRQHandler(void){
	//TIM_Cmd(LPC_TIM0,DISABLE);
	 LPC_TIM2->TCR |= 2;
	if(!(LPC_GPIO3->FIOPIN & (BLUELED_LPC))){
		if(current_blue_pwm_duty < 99){
			LPC_GPIO3->FIOSET |= BLUELED_LPC;
			LPC_TIM2->MR0 = 99 - current_blue_pwm_duty - 1;
		}
	}
	else{
		if(current_blue_pwm_duty > 0){
			LPC_GPIO3->FIOCLR |= BLUELED_LPC;
			LPC_TIM2->MR0 = (uint32_t)(current_blue_pwm_duty);
		}
	}
	//TIM_Cmd(LPC_TIM0,ENABLE);
    LPC_TIM2->TCR &= ~(2);

	TIM_ClearIntPending(LPC_TIM2,TIM_MR0_INT);
}

void TIMER3_IRQHandler(void){
	LPC_TIM3->TCR |= 2;
	switch(led_mode){
		case LED_MODE_FLASH:
			if(!(LPC_GPIO0->FIOPIN & REDLED_LPC)){
				LPC_GPIO0->FIOSET |= REDLED_LPC;
				LPC_GPIO3->FIOCLR |= GREENLED_LPC;
			}
			else if(!(LPC_GPIO3->FIOPIN & GREENLED_LPC)){
				LPC_GPIO3->FIOSET |= GREENLED_LPC;
				LPC_GPIO3->FIOCLR |= BLUELED_LPC;
			}
			else if(!(LPC_GPIO3->FIOPIN & BLUELED_LPC)){
				LPC_GPIO3->FIOSET |= BLUELED_LPC;
				LPC_GPIO0->FIOCLR |= REDLED_LPC;
			}
			break;
		case LED_MODE_STROBE:
			if(strobe_mode_slope){
				current_red_pwm_duty -= 2;
				current_green_pwm_duty -= 2;
				current_blue_pwm_duty -= 2;
				if(current_red_pwm_duty == 1 || current_green_pwm_duty == 1 || current_blue_pwm_duty== 1){
					current_red_pwm_duty -= 1;
					current_green_pwm_duty -= 1;
					current_blue_pwm_duty -= 1;
					strobe_mode_slope = 0;
				}
			}
			else{
				current_red_pwm_duty += 2;
				current_green_pwm_duty += 2;
				current_blue_pwm_duty += 2;
				if(current_red_pwm_duty == 98 || current_green_pwm_duty == 98 || current_blue_pwm_duty== 98){
					current_red_pwm_duty += 1;
					current_green_pwm_duty += 1;
					current_blue_pwm_duty += 1;
					strobe_mode_slope = 1;
				}
			}
			break;
		case LED_MODE_FADE:
			if(fade_mode_state == 0){
				current_red_pwm_duty -= 2;
				current_green_pwm_duty += 2;
				if(current_red_pwm_duty == 1){
					current_red_pwm_duty -= 1;
					current_green_pwm_duty += 1;
					fade_mode_state = 1;
				}
			}
			else if(fade_mode_state == 1){
				current_green_pwm_duty -= 2;
				current_blue_pwm_duty += 2;
				if(current_green_pwm_duty == 1){
					current_green_pwm_duty -= 1;
					current_blue_pwm_duty += 1;
					fade_mode_state = 2;
				}
			}
			else if(fade_mode_state == 2){
				current_blue_pwm_duty -= 2;
				current_red_pwm_duty += 2;
				if(current_blue_pwm_duty == 1){
					current_blue_pwm_duty -= 1;
					current_red_pwm_duty += 1;
					fade_mode_state = 0;
				}
			}
			break;
	}

	LPC_TIM3->TCR &= ~(2);
	TIM_ClearIntPending(LPC_TIM3,TIM_MR0_INT);
	return;
}

void processReceivedData(void){
	LPC_TIM3->TCR |= (2);
	uint8_t info, blueVal, greenVal, redVal;
	blueVal = uartRxBuffer[0];
	greenVal = uartRxBuffer[1];
	redVal = uartRxBuffer[2];
	info = uartRxBuffer[3];

	process_led_mode(info);

	if(LPC_TIM0->TCR & 1 || LPC_TIM1->TCR & 1 || LPC_TIM2->TCR & 1){
		turn_off_PWMs();
	}

	switch(led_mode){
		case LED_MODE_NORMAL:
			set_mode_normal(redVal,greenVal,blueVal);
			break;
		case LED_MODE_FLASH:
			set_mode_flash();
			break;
		case LED_MODE_STROBE:
			set_mode_strobe();
			break;
		case LED_MODE_FADE:
			set_mode_fade();
			break;
	}
	LPC_TIM3->TCR &= ~(2);
}

void set_mode_normal(uint8_t redVal, uint8_t greenVal, uint8_t blueVal){
	conf_PWM_Red(redVal);
	conf_PWM_Green(greenVal);
	conf_PWM_Blue(blueVal);
	turn_on_PWMs();
}

void set_mode_flash(void){

	LPC_TIM3->MR0 = (((10 - led_speed)*1000) - 1);

	GPIO_ClearValue(0, REDLED_LPC);
	GPIO_SetValue(3,(GREENLED_LPC | BLUELED_LPC));

	NVIC_EnableIRQ(TIMER3_IRQn);
	TIM_Cmd(LPC_TIM3,ENABLE);
}

void set_mode_strobe(void){

	LPC_TIM3->MR0 = 800 - 1;//Increment duty cycles every 80[ms]

	conf_PWM_Red(255);
	conf_PWM_Green(255);
	conf_PWM_Blue(255);

	strobe_mode_slope = 1;

	turn_on_PWMs();
	NVIC_EnableIRQ(TIMER3_IRQn);
	TIM_Cmd(LPC_TIM3, ENABLE);
}

void set_mode_fade(void){

	LPC_TIM3->MR0 = 1200 - 1;//Increment duty cycles every 120[ms]

	conf_PWM_Red(255);
	conf_PWM_Green(0);
	conf_PWM_Blue(0);

	fade_mode_state = 0;

	turn_on_PWMs();
	NVIC_EnableIRQ(TIMER3_IRQn);
	TIM_Cmd(LPC_TIM3, ENABLE);
}

void process_led_mode(uint8_t info){
	led_mode = LED_Mode[(info & 0xF)];
	led_speed = LED_Speed[((info & 0xF0)>>4)];
}

void conf_PWM_Red(uint8_t redVal){
	current_red_pwm_duty = ((redVal*100/255) > 0) ? ((redVal*100/255) - 1) : 0;
}

void conf_PWM_Green(uint8_t greenVal){
	current_green_pwm_duty = ((greenVal*100/255) > 0) ? ((greenVal*100/255) - 1) : 0;
}

void conf_PWM_Blue(uint8_t blueVal){
	current_blue_pwm_duty = ((blueVal*100/255) > 0) ? ((blueVal*100/255) - 1) : 0;
}

void turn_on_PWMs(void){
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	TIM_Cmd(LPC_TIM0,ENABLE);
	TIM_Cmd(LPC_TIM1,ENABLE);
	TIM_Cmd(LPC_TIM2,ENABLE);
}

void turn_off_PWMs(void){
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(TIMER1_IRQn);
	NVIC_DisableIRQ(TIMER2_IRQn);

	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER2_IRQn);

	TIM_Cmd(LPC_TIM0,DISABLE);
	TIM_Cmd(LPC_TIM1,DISABLE);
	TIM_Cmd(LPC_TIM2,DISABLE);

	GPIO_SetValue(0, REDLED_LPC);
	GPIO_SetValue(3, (GREENLED_LPC | BLUELED_LPC));
}

void delay(uint32_t times) {
	for(uint32_t i=0; i<times; i++)
		for(uint32_t j=0; j<times; j++);
}
