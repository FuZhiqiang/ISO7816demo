/*
 * ISO7816.c
 *
 *  Created on: Jan 31, 2016
 *      Author: B17589
 */
#include "main.h"
#include "ISO7816.h"

#define PROCEDURE_NULL  0x60
#define PROCEDURE_SW1   0x90
typedef enum{
    IDLE, TRANS_HEADER, TRANS_PROCEDURE, TRANS_DATA, TRANS_SW1, TRANS_SW2
}trans_step_t;
typedef struct{
    trans_step_t step;
    uint8_t *pCmd;
    uint8_t *pSend;
    uint8_t len;
    uint8_t needRspData;
}trans_state_t;

static uint8_t bufIso7816AtrData[MAX_ATR_LENGTH];		//buffer to store the Answer To Reset message from ISO-7816
static uint8_t *pbufIso7816Rx;
static uint32_t ulIso7816Timeout;
static trans_state_t stateTrans = {.step = IDLE,};
static int iApduRspNum;
static uint8_t tpduGetRsp[5] = {0x00, 0xC0, 0x00, 0x00, 0x00};		//TPDU of Get Response (the last byte will be modified when being used)
static TypeRspCallback ResponseCallback;

const uint8_t apduTest1[] = {0x00, 0xA4, 0x00, 0x00, 0x02, 0x10, 0x01};	//APDU for test,
	//expected response is: 6F 32 84 07 50 41 59 2E53 5a 54 a5 27 9f 08 01 02 9f 0c 20
	//00 00 00 00 00 00 00 00 FD 20 00 00 51 80 00 00 4 D0 12 13 20 09 03 03 20 19 03 03 10 10 00 00 90 00
const uint8_t apduTest2[] = {0x80, 0x5C, 0x00, 0x02, 0x04}; //APDU for test,
	//expected response is: 5C 00 00 41 00 90 00
const uint8_t apduGetC1[]={0x00, 0x84, 0x00, 0x00, 0x04};	//APDU of Get Challenge (get a 32-bit random number from IC card)

//Below definitions are for hardware initialization
#define FSL_LPUARTCOM1	0
#define FSL_TPMTMR1		1

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

uint32_t ulUartRxTime;

extern void LPUART_DRV_IRQHandler(uint32_t instance);
void LPUART0_IRQHandler(void)
{
    LPUART_DRV_IRQHandler(FSL_LPUARTCOM1);
}

void lpuartCom1_RxCallback(uint32_t instance, void * lpuartState)
{
	lpuart_state_t* puartState;

	puartState = (lpuart_state_t*)lpuartState;

	ulUartRxTime = OSA_TimeGetMsec();

    Iso7816TransT0Proc(*(puartState->rxBuff), pbufIso7816Rx);

	++puartState->rxBuff;

	if(++puartState->rxSize >= BUFF_LENGTH)
	{
		Iso7816StartRxNextFrame(pbufIso7816Rx);
	}
}

void lpuartCom1_TxCallback(uint32_t instance, void * lpuartState)
{
	lpuart_state_t* puartState;

	puartState = (lpuart_state_t*)lpuartState;

    ++puartState->txBuff;
    --puartState->txSize;
}

uint8_t* Iso7816GetAtrData(void)
{
	return bufIso7816AtrData;
}

void Iso7816StartRxNextFrame(uint8_t *bufUartRx)
{
	lpuartCom1_State.rxBuff = bufUartRx;
	lpuartCom1_State.rxSize = 0;
	ulUartRxTime = 0;
}

void Iso7816Init(TypeRspCallback ApduRspCallback)
{
	//init UART0 to ISO7816 mode, enable PTB1/UART0_TX pin internal pull up
	LPUART_DRV_Init(FSL_LPUARTCOM1,&lpuartCom1_State,&lpuartCom1_InitConfig0);
	LPUART_DRV_InstallRxCallback(FSL_LPUARTCOM1, lpuartCom1_RxCallback, bufIso7816AtrData, NULL, true);
	LPUART_DRV_InstallTxCallback(FSL_LPUARTCOM1, lpuartCom1_TxCallback, NULL, NULL);
	PORT_HAL_SetMuxMode(PORTB,1UL,kPortMuxAlt2);
	PORT_HAL_SetPullMode(PORTB, 1, kPortPullUp);
	PORT_HAL_SetPullCmd(PORTB, 1, true);
	SIM_HAL_SetLpuartOpenDrainCmd(SIM, 0, true);	//set UART0_TX to open drain mode
    LPUART_HAL_SetSingleWireCmd(LPUART0, true);	    //set receiver input from UART0_TX
    LPUART_HAL_SetTxdirInSinglewireMode(LPUART0, kLpuartSinglewireTxdirIn);	//set it to receive mode

    PORT_HAL_SetMuxMode(PORTA,12UL,kPortMuxAlt2);
    SIM_HAL_SetTpmChSrcMode(SIM,TPM1_IDX,0,kSimTpmChSrc0);
	TPM_DRV_Init(FSL_TPMTMR1, &tpmTmr1_InitConfig0);
	TPM_DRV_SetClock(FSL_TPMTMR1, kTpmClockSourceModuleHighFreq, kTpmDividedBy1);
    //enable channel 1 to output PWM to LED, for debug only
    PORT_HAL_SetMuxMode(PORTB,13UL,kPortMuxAlt2);
	TPM_DRV_PwmStart(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 1U);
  
    PORT_HAL_SetMuxMode(PORTA,8UL,kPortMuxAsGpio);
	GPIO_DRV_Init(NULL, gpio1_OutConfig0);

	ResponseCallback = ApduRspCallback;
}

int Iso7816Deactivate(void)
{
	GPIO_DRV_ClearPinOutput(RST);
	OSA_TimeDelay(1);
	TPM_DRV_PwmStop(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 0U);
	return 0;
}

int Iso7816Activate(uint8_t *bufUartRx)
{
	int i;

    pbufIso7816Rx = bufUartRx;
	GPIO_DRV_ClearPinOutput(RST);
	TPM_DRV_PwmStart(FSL_TPMTMR1, &tpmTmr1_ChnConfig0, 0U);
	OSA_TimeDelay(2);
	Iso7816StartRxNextFrame(bufIso7816AtrData);
	GPIO_DRV_SetPinOutput(RST);
	//After RST is released to logic high, the ATR should come from the card within a 400/f and 40000/f period of time.
	OSA_TimeDelay(40);
	if(ulUartRxTime)
	{
		while(20 > (OSA_TimeGetMsec() - ulUartRxTime))
		{
			;		//wait to IC card Answer To Reset complete
		}
		Iso7816StartRxNextFrame(bufUartRx);
		return 0;	//success
	}
	else
	{	// If the ATR does not come, the reader should apply a deactivation.
		Iso7816Deactivate();
		return 1;	//failed
	}
}

int Iso7816SendCmd(uint8_t *apduCmd, uint8_t length)
{
    assert(apduCmd);
    assert(length >= 4);

    if(4 == length)
    {
    	apduCmd[4] = 0;
    	length++;
    }
    else if(length > 5)
    {
    	if(apduCmd[4] < (length - 5))
    	{
    		stateTrans.needRspData = apduCmd[length - 1];
    		length--;
    	}
    }

    if(IDLE == stateTrans.step)
    {
        stateTrans.step = TRANS_HEADER;
        stateTrans.pCmd = apduCmd;
        LPUART_DRV_SendData(FSL_LPUARTCOM1, apduCmd, 5);
        stateTrans.pSend = apduCmd + 5;
        stateTrans.len = length - 5;
        stateTrans.step = TRANS_PROCEDURE;
        ulIso7816Timeout = OSA_TimeGetMsec() + ISO7816_MAX_WAIT_TIME_MS;
    }
    else
    {
        return 1;
    }

    return 0;
}

void Iso7816TransT0Proc(uint8_t charRx, uint8_t *apduResponse)
{
	uint8_t sw1, len;

    switch(stateTrans.step)
    {
    case TRANS_PROCEDURE:
        if(PROCEDURE_NULL == charRx)
        {
            return;
        }
        else if((PROCEDURE_SW1 == (charRx & 0xF0))
                || (PROCEDURE_NULL == (charRx & 0xF0)))
        {
            apduResponse[iApduRspNum] = charRx;
            iApduRspNum++;
            stateTrans.step = TRANS_SW2;
        }
        else if(charRx == stateTrans.pCmd[1])
        {
            if(stateTrans.len)
            {
                LPUART_DRV_SendData(FSL_LPUARTCOM1, stateTrans.pSend, stateTrans.len);
                stateTrans.len = 0;
                stateTrans.step = TRANS_SW1;
            }
            else if(stateTrans.pCmd[4])
            {
                stateTrans.step = TRANS_DATA;
            }
            else
            {
                stateTrans.step = TRANS_SW1;
            }
        }
        else if(charRx == (~stateTrans.pCmd[1]))
        {
            if(stateTrans.len)
            {
                LPUART_DRV_SendData(FSL_LPUARTCOM1, stateTrans.pSend, 1);
                stateTrans.pSend++;
                stateTrans.len--;
            }
        }
        else
        {
            apduResponse[iApduRspNum] = charRx;
            iApduRspNum++;
            stateTrans.step = TRANS_SW2;
        }
        break;
    case TRANS_DATA:
        apduResponse[iApduRspNum] = charRx;
        iApduRspNum++;
        if(stateTrans.pCmd[4] == iApduRspNum)
        {
            stateTrans.step = TRANS_SW1;
        }
        break;
    case TRANS_SW1:
        apduResponse[iApduRspNum] = charRx;
        iApduRspNum++;
        stateTrans.step = TRANS_SW2;
        break;
    case TRANS_SW2:
        apduResponse[iApduRspNum] = charRx;
    	sw1 = apduResponse[iApduRspNum-1];
    	len = iApduRspNum + 1;
        iApduRspNum = 0;
	    stateTrans.step = IDLE;
    	if(0x61 == sw1)
    	{
    		tpduGetRsp[4] = charRx;
    		Iso7816SendCmd(tpduGetRsp, 5);
    	}
    	else if(0x6C == sw1)
    	{
    		stateTrans.pCmd[4] = charRx;
    		Iso7816SendCmd(stateTrans.pCmd, 5);
    	}
    	else if((0x90 == sw1) && (0x00 == charRx))
    	{
    		if(stateTrans.needRspData)
    		{
    			tpduGetRsp[4] = stateTrans.needRspData;
    			stateTrans.needRspData = 0;
    			Iso7816SendCmd(tpduGetRsp, 5);
    		}
    		else
    		{
                ResponseCallback(apduResponse, len);
    		}
    	}
    	else
    	{
            ResponseCallback(apduResponse, len);
    	}
    	break;
    default:
	    stateTrans.step = IDLE;
    }
}

void Iso7816CheckTimeout(void)
{
    if(IDLE != stateTrans.step)
    {
        if(ulUartRxTime)	//If there is data come from UART
        {
            ulIso7816Timeout = ulUartRxTime + ISO7816_MAX_WAIT_TIME_MS;
            ulUartRxTime = 0;
        }
        else if(ulIso7816Timeout < OSA_TimeGetMsec())
        {
            Iso7816StartRxNextFrame(pbufIso7816Rx);
            stateTrans.step = IDLE;
        }
    }
	//Test ISO7816 "get challenge" and other commands
	else
	{
	//Iso7816SendCmd((uint8_t*)apduGetC1, 5);
	//Iso7816SendCmd((uint8_t*)apduTest1, 7);
	//Iso7816SendCmd((uint8_t*)apduTest2, 5);
	}
}
