/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////

// SDK Included Files
#include "main.h"
#include "ISO7816.h"

/* Configuration for enter RUN mode. Core clock = 48MHz. */
const clock_manager_user_config_t g_defaultClockConfigRun =
{
    .mcgliteConfig =
    {
        .mcglite_mode        = kMcgliteModeHirc48M,   // Work in HIRC mode.
        .irclkEnable        = true,     // MCGIRCLK disable.
        .irclkEnableInStop  = false,    // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgliteLircSel2M,    // Select LIRC_2M.
        .fcrdiv             = kMcgliteLircDivBy1,    // FCRDIV is 0.
        .lircDiv2           = kMcgliteLircDivBy1,    // LIRC_DIV2 is 0.
        .hircEnableInNotHircMode         = true,  // HIRC disable.
    },
    .simConfig =
    {
        .er32kSrc  = kClockEr32kSrcOsc0,  // ERCLK32K selection, use OSC.
        .outdiv1   = 0U,
        .outdiv4   = 1U,
    },
    .oscerConfig =
    {
        .enable       = false, // OSCERCLK disable.
        .enableInStop = false, // OSCERCLK disable in STOP mode.
    }
};

const uint8_t apduGetC1[]={0x00, 0x84, 0x00, 0x00, 0x04};	//APDU of Get Challenge Command(get a 32-bit random number from IC card)
uint8_t apduResponse[256];

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
}trans_state_t;
trans_state_t stateTrans = {.step = IDLE,};

int SendCommand(uint8_t *apduCmd, uint8_t length);
void ResponseCallback(uint8_t *apduRsp, uint8_t length);

int main(void)
{
    uint32_t iTimeout;
    
	SIM_HAL_EnableClock(SIM,kSimClockGatePortA);
	SIM_HAL_EnableClock(SIM,kSimClockGatePortB);
    CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
	OSA_TimeInit();

	Iso7816Init();
	Iso7816Activate();

    if(0 == SendCommand((uint8_t*)apduGetC1, 5))
    {
        iTimeout = OSA_TimeGetMsec() + iMaxWaitTimeMs;
    }
    
    while(true)
    {
        if(IDLE != stateTrans.step)
        {
            if(iUartRxTime)	//If there is data come from UART
            {
                iTimeout = iUartRxTime + iMaxWaitTimeMs;
                iUartRxTime = 0;
            }
            else if(iTimeout < OSA_TimeGetMsec())
            {
                UartStartRxNextFrame();
                stateTrans.step = IDLE; 
            }
        }
    }

}

int SendCommand(uint8_t *apduCmd, uint8_t length)
{
    assert(apduCmd);
    assert(length >= 5);
    
    if(IDLE == stateTrans.step)
    {
        stateTrans.step = TRANS_HEADER;
        stateTrans.pCmd = apduCmd;
        LPUART_DRV_SendData(FSL_LPUARTCOM1, apduCmd, 5);
        stateTrans.pSend = apduCmd + 5;
        stateTrans.len = length - 5;
        stateTrans.step = TRANS_PROCEDURE;
    }
    else
    {
        return 1;
    }
    
    return 0;
}

void TransT0Proc(uint8_t charRx)
{
    static int i;

    if(TRANS_PROCEDURE == stateTrans.step)
    {
        if(PROCEDURE_NULL == charRx)
        {
            return;
        }
        else if((PROCEDURE_SW1 == (charRx & 0xF0))
                || (PROCEDURE_NULL == (charRx & 0xF0)))
        {
            apduResponse[i] = charRx;
            i++;
            stateTrans.step = TRANS_SW2;
        }
        else if(charRx == stateTrans.pCmd[1])
        {
            if(stateTrans.len)
            {
                LPUART_DRV_SendData(FSL_LPUARTCOM1, stateTrans.pSend, stateTrans.len);
                stateTrans.len = 0;
            }
            if(stateTrans.pCmd[4])
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
    }
    else if(TRANS_DATA == stateTrans.step)
    {
        apduResponse[i] = charRx;
        i++;
        if(stateTrans.pCmd[4] == i)
        {
            stateTrans.step = TRANS_SW1;
        }
    }
    else if(TRANS_SW1 == stateTrans.step)
    {
        apduResponse[i] = charRx;
        i++;
        stateTrans.step = TRANS_SW2;
    }
    else if(TRANS_SW2 == stateTrans.step)
    {
        apduResponse[i] = charRx;
        ResponseCallback(apduResponse, i+1);
        i = 0;
        stateTrans.step = IDLE;
    }
}

void ResponseCallback(uint8_t *apduRsp, uint8_t length)
{
    //write application codes here to deal with the response APDU
}
