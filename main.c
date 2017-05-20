#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"
#include "led_corr.h"
#include <stdlib.h>

#define LEDPORT GPIOD
#define LED1 GPIO_Pin_13
#define LED2 GPIO_Pin_14
#define LED3 GPIO_Pin_15
#define LED4 GPIO_Pin_12
#define BUTTONPORT GPIOA
#define BUTT1 GPIO_Pin_0

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

typedef struct
{
	u8 state, check, temp;
	u32 time;
} buttonInfo;

buttonInfo butt0;
buttonInfo butt1;

u32 ms, downCount, usbTime, simTime, ledOffTime;
volatile u16 adcBuffer[3];
volatile u16 set, control, output0, output1;
u8 regulation = 0, identification;
float simPrev[2] = {0, 0};
float simNext[2] = {0, 0};
volatile u32 rngNum;
char tab[64];
u16 simBuff[1000];
u16 simDelay = 50; // 0.5 s
u16 identVals[13] = {0, 511, 1023, 1535, 2047, 2559, 3071, 2559, 2047, 1535, 1023, 511, 0};
u16 identStep = 0;
u16 identStepTime = 8000;
u32 identTime;
float h = 0.01f;
float yDiscreteBuff[2] = {0,0};
float uDiscreteBuff[2] = {0,0};
float yDiscrete;
float k = 0.83;
float T1 = 0.15, T2 = 1.35;
float num[3], den[3];

void initBuff(u16 val)
{
	int i;
	for (i = 0;i < simDelay;i++)
		simBuff[i] = val;
}

u16 pushToBuff(u16 val)
{
	int i;
	u16 out = simBuff[simDelay - 1];
	for (i = simDelay - 1;i > 0;i--)
		simBuff[i] = simBuff[i - 1];
	simBuff[0] = val;
	return out;
}

void ColorfulRingOfDeath(void)
{
	while (1)
	{
		uint32_t count = 0;
		while (count++ < 300000);
		GPIO_ToggleBits(LEDPORT, LED1 | LED3);
	}
}


void stateEqs(float* differentials, float* state, float* u)
{
	differentials[0] = state[1];
	differentials[1] = -(T1 + T2) / (T1 * T2) * state[1] - 1 / (T1 * T2) * state[0] + k / (T1 * T2) * (*u);
}

void rk4(float* next, float* prev, float* u, float* h)
{
	float k1[2], k2[2], k3[2], k4[2];
	float temp[2];
	// k1
	stateEqs(k1, prev, u);
	k1[0] = (*h) * k1[0]; k1[1] = (*h) * k1[1];
	// k2
	temp[0] = prev[0] + 0.5 * k1[0];
	temp[1] = prev[1] + 0.5 * k1[1];
	stateEqs(k2, temp, u);
	k2[0] = (*h) * k2[0]; k2[1] = (*h) * k2[1];
	// k3
	temp[0] = prev[0] + 0.5 * k2[0];
	temp[1] = prev[1] + 0.5 * k2[1];
	stateEqs(k3, temp, u);
	k3[0] = (*h) * k3[0]; k3[1] = (*h) * k3[1];
	// k4
	temp[0] = prev[0] + k3[0];
	temp[1] = prev[1] + k3[1];
	stateEqs(k4, temp, u);
	k4[0] = (*h) * k4[0]; k4[1] = (*h) * k4[1];
	// result
	next[0] = prev[0] + 1.0 / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
	next[1] = prev[1] + 1.0 / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
}

void sim(u16 u)
{
	float z = 1.0 * u;
	rk4(simNext, simPrev, &z, &h);
	simPrev[0] = simNext[0];
	simPrev[1] = simNext[1];
}

void simDiscreteInit()
{
	float A = T1 * T2 * 4 / (h * h);
	float B = (T1 + T2) * 2 / h;
	num[0] = k;
	num[1] = 2 * k;
	num[2] = k;
	den[0] = A + B + 1;
	den[1] = -2 * A + 2;
	den[2] = A - B + 1;
}

void simDiscrete(u16 u)
{
	float z = 1.0 * u;
	yDiscrete = -den[1] / den[0] * yDiscreteBuff[0] - den[2] / den[0] * yDiscreteBuff[1] +
			num[0] / den[0] * z + num[1] / den[0] * uDiscreteBuff[0] + num[2] / den[0] * uDiscreteBuff[0];
	yDiscreteBuff[1] = yDiscreteBuff[0];
	yDiscreteBuff[0] = yDiscrete;
	uDiscreteBuff[1] = uDiscreteBuff[0];
	uDiscreteBuff[0] = z;
}

float sumE = 0;
float lastE = 0;
u16 pid(u16 sv, u16 pv, float* h)
{
	u16 out;
	float e, c;
	//float kp = 230.4107, ti = 1.1494, td = 0.05268;
	//float kp = 10, ti = 0.5, td = 0;
	float kp = 12, ti = 0.2, td = 0.5;
	u8 enableI = 1;
	u8 enableD = 1;

	e = sv - pv;
	c = kp * e;
	if (enableI)
	{
		sumE += e;
		c += kp / ti * (*h) * sumE;
	}
	if (enableD)
	{
		c += kp * td * (e - lastE);
		lastE = e;
	}
	if (c > 4095)
	{
		c = 4095;
		sumE -= e;
	}
	else if (c < 0)
	{
		c = 0;
		sumE -= e;
	}
	return (u16)c;
}

u8 state = 0;
u16 relay(u16 sv, u16 pv)
{
	u16 outOff = 0, outOn = 4095, out;
	int on = 100, off = -100;
	u8 newState;
	int e = sv - pv;
	if (e < off)
		newState = 0;
	else if (e > on)
		newState = 3;
	else if (e <= on && e >= off)
	{
		if (state == 0 || state == 1)
			newState = 1;
		else if (state == 2 || state == 3)
			newState = 2;
	}

	if (newState == 0 || newState == 1)
		out = outOff;
	else
		out = outOn;
	state = newState;
	return out;
}

void initGpio()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// PD13 - dioda wskazujaca open/close loop
	// PD15 - dioda migajaca przy transmisji
	// PD14 -
	// PD12 -
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED1 | LED3 | LED2 | LED4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LEDPORT, &GPIO_InitStruct);

	// PC8 - dioda wskazujaca wartosc wejscia
	// PC9 - dioda wskazujaca wartosc wyjscia
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

	// PA0 - wejscie przycisku
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = BUTT1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(BUTTONPORT, &GPIO_InitStruct);

	// PE4 - wejscie przycisku z plytki
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	// PA1 - ADC1_CH1 - wejscie z potencjometru
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA2 - ADC2_CH2 - pomiar napiecia na kondensatorze, wyjscie
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA3 - ADC2_CH3 - pomiar napiecia na kondensatorze, wyjscie
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PE5 - pin sterujacy
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
}

void initAdc()
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStruct;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 3;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_480Cycles);
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	ADC_DMACmd(ADC1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = 0;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcBuffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_Init(&NVIC_InitStructure);

	ADC_SoftwareStartConv(ADC1);
}

void initPwm()
{
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	TIM_OCInitTypeDef TIM_OcInitStruct = {0,};

	// PWMy na LEDy
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InitStruct.TIM_Prescaler = 21 - 1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Period = 4096 - 1;
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_InitStruct);
	TIM_Cmd(TIM3, ENABLE);

	TIM_OcInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OcInitStruct.TIM_Pulse = 400;
	TIM_OcInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OcInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM3, &TIM_OcInitStruct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM_OcInitStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// PWM sterujacy
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	TIM_InitStruct.TIM_Prescaler = 42 - 1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Period = 4096 - 1;
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9, &TIM_InitStruct);
	TIM_Cmd(TIM9, ENABLE);

	TIM_OcInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OcInitStruct.TIM_Pulse = 400;
	TIM_OcInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OcInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &TIM_OcInitStruct);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
}

void initRng()
{
	RCC_AHB2PeriphClockCmd(RCC_AHB2ENR_RNGEN, ENABLE);
	RNG_ITConfig(ENABLE);
	RNG_Cmd(ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = HASH_RNG_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void delay(uint32_t ms)
{
	downCount = ms;
	while(downCount != 0);
}

int main(void)
{
	SystemInit();
	initGpio();
	initAdc();
	initPwm();
	initRng();

	initBuff(0);
	simDiscreteInit();

	if (SysTick_Config(SystemCoreClock / 1000))
		ColorfulRingOfDeath();
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);

	while (1)
	{
		if (ms - simTime >= 10)
		{
			set = adcBuffer[0];
			output0 = adcBuffer[1];
			output1 = adcBuffer[2];

			// sztuczny szum do pomiaru
			/*int x = output0 + ((rngNum % 50) - 25);
			if (x > 4095)
				x = 4095;
			else if (x < 0)
				x = 0;
			output0 = x;*/

			// opoznienie na tablicy
			//output0 = pushToBuff(output0);

			if (identification == 1)
			{
				control = identVals[identStep];
			}
			else
			{
				float dt = 0.01;
				if (regulation == 0)
					control = set;
				else if (regulation == 1)
					control = pid(set, output0, &dt);
				else if (regulation == 2)
					control = relay(set, output0);
			}


			TIM_SetCompare3(TIM3, led[set]);
			TIM_SetCompare1(TIM9, control);
			TIM_SetCompare4(TIM3, led[output0]);

			sim(control);
			simDiscrete(control);

			simTime = ms;
		}

		butt0.temp = GPIO_ReadInputDataBit(BUTTONPORT, BUTT1);
		if (butt0.check)
		{
			if (butt0.state == 0 && butt0.temp == 1)
			{
				butt0.time = ms + 50;
				butt0.check = 0;
			}
		}
		else
		{
			if (ms >= butt0.time && butt0.temp == 1)
			{
				regulation++;
				regulation = regulation % 3;
				if (regulation != 0)
					GPIO_SetBits(LEDPORT, LED1);
				else
					GPIO_ResetBits(LEDPORT, LED1);
				butt0.check = 1;
			}
		}
		butt0.state = butt0.temp;


		if (butt1.check)
		{
			butt1.temp = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
			if (butt1.state == 1 && butt1.temp == 0)
			{
				butt1.time = ms + 50;
				butt1.check = 0;
			}
			butt1.state = butt1.temp;
		}
		else
		{
			if (ms >= butt1.time && GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0)
			{
				GPIO_SetBits(GPIOD, LED2 | LED4);

				identification = 1;
				identStep = 0;
				identTime = ms + identStepTime;

				butt1.check = 1;
			}
		}

		if (ms >= identTime)
		{
			identStep++;
			if (identStep > 12)
			{
				GPIO_ResetBits(GPIOD, LED2 | LED4);
				identification = 0;
			}
			else
				identTime = ms + identStepTime;
		}

		if (ms - usbTime >= 100)
		{
			GPIO_SetBits(LEDPORT, LED3);
			sprintf(tab,"%010lu|%01u|%04u|%04u|%04u|%04u|%04u", ms, regulation, set, control, output0, output1, (u16)yDiscrete);
			VCP_send_str((uint8_t*)tab);

			ledOffTime = ms;
			usbTime = ms;
		}

		if (ms - ledOffTime >= 10)
			GPIO_ResetBits(LEDPORT, LED3);

	}

	return 0;
}

void SysTick_Handler(void)
{
	ms++;
	if (downCount > 0)
		downCount--;
}

void ADC_IRQHandler()
{
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}

}

void HASH_RNG_IRQHandler()
{
	if (RNG_GetFlagStatus(RNG_FLAG_DRDY) != RESET)
		rngNum = RNG_GetRandomNumber();
}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}

void HardFault_Handler(void) { ColorfulRingOfDeath(); }
void MemManage_Handler(void) { ColorfulRingOfDeath(); }
void BusFault_Handler(void)  { ColorfulRingOfDeath(); }
void UsageFault_Handler(void){ ColorfulRingOfDeath(); }
