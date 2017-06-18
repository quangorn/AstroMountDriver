#ifndef __LOGIC__ENCODER_H_
#define __LOGIC__ENCODER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void EqUpdateEncoderValues(void);
void EqGetEncoderValues(uint16_t* pValueX, uint16_t* pValueY);

#ifdef __cplusplus
}
#endif
	 
#endif
