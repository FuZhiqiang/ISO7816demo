#ifndef INCLUDED_ISO7816_H
#define INCLUDED_ISO7816_H

#define FSL_LPUARTCOM1	0
#define FSL_TPMTMR1		1

#define BUFF_LENGTH		64
extern uint8_t uartRxBuff[BUFF_LENGTH];
extern uint32_t iUartRxTime, iMaxWaitTimeMs;

void UartStartRxNextFrame(void);
void Iso7816Init(void);
void Iso7816Deactivate(void);
int Iso7816Activate(void);
uint32_t OSA_TimeDiff(uint32_t time_start, uint32_t time_end);

#endif //INCLUDED_ISO7816_H
