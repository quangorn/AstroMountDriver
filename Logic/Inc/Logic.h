#ifndef __LOGIC__LOGIC_H_
#define __LOGIC__LOGIC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void InitLogic(void);
void EqProcessReceive(uint8_t* pRecvBuf);
	 
#ifdef __cplusplus
}
#endif
	 
#endif
