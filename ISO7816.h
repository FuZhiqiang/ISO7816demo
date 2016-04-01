/*
 * ISO7816.h
 *
 *  Created on: Jan 31, 2016
 *      Author: B17589
 */

#ifndef SOURCES_ISO7816_H_
#define SOURCES_ISO7816_H_

#define ISO7816_MAX_WAIT_TIME_MS	1000
#define MAX_ATR_LENGTH	    33
#define BUFF_LENGTH        260

typedef void (*TypeRspCallback)(uint8_t *apduRsp, uint8_t length);

void Iso7816StartRxNextFrame(uint8_t *bufUartRx);
void Iso7816Init(TypeRspCallback ApduRspCallback);
int Iso7816Deactivate(void);
int Iso7816Activate(uint8_t *bufUartRx);
int Iso7816SendCmd(uint8_t *apduCmd, uint8_t length);
void Iso7816TransT0Proc(uint8_t charRx, uint8_t *apduResponse);
void Iso7816CheckTimeout(void);
uint8_t* Iso7816GetAtrData(void);

#endif /* SOURCES_ISO7816_H_ */
