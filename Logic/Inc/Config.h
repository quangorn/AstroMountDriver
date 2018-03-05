#ifndef __LOGIC__CONFIG_H_
#define __LOGIC__CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

void EqReadConfig(void* ptr);
int EqWriteConfig(void* ptr);

void EqReadEncoderCorrection(int pageCount, void* ptr);
int EqWriteEncoderCorrection(int pageCount, void* ptr);
int EqClearEncoderCorrection();
	 
void EqReadPEC(int pageCount, void* ptr);
int EqWritePEC(int pageCount, void* ptr);
int EqClearPEC();

#ifdef __cplusplus
}
#endif
	 
#endif
