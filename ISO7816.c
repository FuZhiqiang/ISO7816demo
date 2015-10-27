#include "main.h"
#include "ISO7816.h"

#define MAX_ATR_LENGTH	33
char dataAtr[MAX_ATR_LENGTH];		//buffer to store the Answer To Reset message from ISO-7816

enum _gpio1_pinNames{
  RST = GPIO_MAKE_PIN(GPIOA_IDX, 8U),
};

const tpm_general_config_t tpmTmr1_InitConfig0 = {
  .isDBGMode = true,
  .isGlobalTimeBase = false,
  .isTriggerMode = false,
  .isStopCountOnOveflow = false,
  .isCountReloadOnTrig = false,
  .triggerSource = kTpmTrigSel0,
};
tpm_pwm_param_t tpmTmr1_ChnConfig0 = {
  .mode = kTpmEdgeAlignedPWM,
  .edgeMode = kTpmHighTrue,
  .uFrequencyHZ = 3571200U,
  .uDutyCyclePercent = 50U,
};

const lpuart_user_config_t lpuartCom1_InitConfig0 = {
  .clockSource = kClockLpuartSrcIrc48M,
  .baudRate = 9600U,
  .parityMode = kLpuartParityEven,
  .stopBitCount = kLpuartTwoStopBit,
  .bitCountPerChar = kLpuart9BitsPerChar,
};
lpuart_state_t lpuartCom1_State;

const gpio_output_pin_user_config_t gpio1_OutConfig0[] = {
  {
    .pinName = RST,
    .config.outputLogic = 0,
    .config.slewRate = kPortFastSlewRate,
    .config.driveStrength = kPortLowDriveStrength,
  },
  {
    .pinName = GPIO_PINS_OUT_OF_RANGE,
  }
};

uint8_t uartRxBuff[BUFF_LENGTH];
uint32_t iUartRxTime, iMaxWaitTimeMs = 1000;

extern void LPUART_DRV_IRQHandler(uint32_t instance);
void LPUART0_IRQHandler(void)
{
    LPUART_DRV_IRQHandler(FSL_LPUARTCOM1);
}

void UartStartRxNextFrame(void)
{
	lpuartCom1_State.rxBuff = uartRxBuff;
	lpuartCom1_State.rxSize = 0;
	iUartRxTime = 0;
}

void lpuartCom1_RxCallback(uint32_t instance, void * lpuartState)
{
	lpuart_state_t* puartState;

	puartState = (lpuart_state_t*)lpuartState;

    TransT0Proc(*(puartState->rxBuff));

	++puartState->rxBuff;

	if(++puartState->rxSize >= BUFF_LENGTH)
	{
		UartStartRxNextFrame();
	}
    
	iUartRxTime = OSA_TimeGetMsec();
}

void lpuartCom1_TxCallback(uint32_t instance, void * lpuartState)
{
	lpuart_state_t* puartState;

	puartState = (lpuart_state_t*)lpuartState;

    ++puartState->txBuff;
    --puartState->txSize;
}

void Iso7816Init(void)
{
	//init UART0 to ISO7816 mode, enable PTB1/UART0_TX pin internal pull up
	LPUART_DRV_Init(FSL_LPUARTCOM1,&lpuartCom1_State,&lpuartCom1_InitConfig0);
	LPUART_DRV_InstallRxCallback(FSL_LPUARTCOM1, lpuartCom1_RxCallback, uartRxBuff, NULL, true);
	LPUART_DRV_InstallTxCallback(FSL_LPUARTCOM1, lpuartCom1_TxCallback, NULL, NULL);
	PORT_HAL_SetMuxMode(PORTB,1UL,kPortMuxAlt2);
	PORT_HAL_SetPullMode(PORTB, 1, kPortPullUp);
	PORT_HAL_SetPullCmd(PORTB, 1, true);
	SIM_SOPT5 |= SIM_SOPT5_LPUART0ODE_MASK;		//set UART0_TX to open drain mode
	LPUART0_CTRL |= (LPUART_CTRL_LOOPS_MASK | LPUART_CTRL_RSRC_MASK);	//set receiver input from UART0_TX
	LPUART0_CTRL &= (~LPUART_CTRL_TXDIR_MASK);	//set it to receive mode

    PORT_HAL_SetMuxMode(PORTA,12UL,kPortMuxAlt2);
    SIM_HAL_SetTpmChSrcMode(SIM,TPM1_IDX,0,kSimTpmChSrc0);
	TPM_DRV_Init(FSL_TPMTMR1, &tpmTmr1_InitConfig0);
	TPM_DRV_SetClock(FSL_TPMTMR1, kTpmClockSourceModuleHighFreq, kTpmDividedBy1);
    //enable channel 1 to output PWM to LED, for debug only
    PORT_HAL_SetMuxMode(PORTB,13UL,kPortMuxAlt2);
	TPM_DRV_PwmStart(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 1U);
  
    PORT_HAL_SetMuxMode(PORTA,8UL,kPortMuxAsGpio);
	GPIO_DRV_Init(NULL, gpio1_OutConfig0);
}

void Iso7816Deactivate(void)
{
	GPIO_DRV_ClearPinOutput(RST);
	OSA_TimeDelay(1);
	TPM_DRV_PwmStop(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 0U);
}

int Iso7816Activate(void)
{
	int i;

	GPIO_DRV_ClearPinOutput(RST);
	TPM_DRV_PwmStart(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 0U);
	OSA_TimeDelay(2);
	UartStartRxNextFrame();
	GPIO_DRV_SetPinOutput(RST);
	//After RST is released to logic high, the ATR should come from the card within a 400/f and 40000/f period of time.
	OSA_TimeDelay(20);
	if(iUartRxTime)
	{
		while(iMaxWaitTimeMs >= OSA_TimeDiff(iUartRxTime, OSA_TimeGetMsec()))
		{
			;		//wait to IC card Answer To Reset complete
		}
		if(lpuartCom1_State.rxSize > MAX_ATR_LENGTH)
		{
			lpuartCom1_State.rxSize = MAX_ATR_LENGTH;
		}
		for(i = 0; i < lpuartCom1_State.rxSize; i++)
		{
			dataAtr[i] = uartRxBuff[i];	//copy ATR message from UART receive buffer
		}
		UartStartRxNextFrame();
	}
	else
	{	// If the ATR does not come, the reader should apply a deactivation.
		Iso7816Deactivate();
        return 1;
	}
    
    return 0;
}
