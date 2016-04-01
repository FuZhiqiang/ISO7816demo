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

void MyApduRspCallback(uint8_t *apduRsp, uint8_t length);
uint8_t bufUart0Rx[BUFF_LENGTH];

int main(void)
{
    uint32_t iTimeout;
    
	SIM_HAL_EnableClock(SIM,kSimClockGatePortA);
	SIM_HAL_EnableClock(SIM,kSimClockGatePortB);
    CLOCK_SYS_SetConfiguration(&g_defaultClockConfigRun);
	OSA_TimeInit();

    Iso7816Init(MyApduRspCallback);
    Iso7816Activate(bufUart0Rx);
    
    while(true)
    {
        Iso7816CheckTimeout();
    }
}

void MyApduRspCallback(uint8_t *apduRsp, uint8_t length)
{
    //write application codes here to deal with the response APDU
	Iso7816StartRxNextFrame(bufUart0Rx);
}
