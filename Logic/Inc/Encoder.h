#ifndef __LOGIC__ENCODER_H_
#define __LOGIC__ENCODER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void EqUpdateEncoderValues(void);
void EqGetEncoderValues(int16_t* pValueX, int16_t* pValueY);

#ifdef __cplusplus
}
#endif
	 
#endif
