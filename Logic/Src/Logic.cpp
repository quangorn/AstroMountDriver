#include <main.h>
#include <gpio.h>
#include <string.h>
#include <tim.h>
#include <Protocol.h>
#include "Encoder.h"
#include "Config.h"
#include "Logic.h"

using namespace EQ;

Config m_Config;

uint32_t m_nMicrostepCount[2] = {0, 0};
bool m_lMotorEnabled[2] = {false, false};
bool m_lMotorDisableScheduled[2] = {false, false};
uint8_t m_nMotorDirection[2] = {DIR_FORWARD, DIR_FORWARD};
uint32_t m_nMotorTargetRate[2] = {1, 1}; //1 = Sidereal
uint32_t m_nMotorCurrentRate[2] = {0, 0};
uint32_t m_nMotorChangeSpeedMicrostep[2] = {0, 0};
uint32_t m_nMotorLastAdjustedSpeedMicrostep[2] = {0, 0};
uint16_t m_nTimerDMA_Period[2] = {1, 1}; //How many microsteps per DMA interrupt

bool m_lGoToEnabled[2] = {false, false}; //If GoTo command is in process
bool m_lGoToSlowDownAdjusted[2] = {false, false}; //If we moved slowdown point due to reaching full speed
bool m_lGoToSlowedDown[2] = {false, false}; //If we already passed slow down point during GoTo
uint32_t m_nGoToStartMicrostep[2]; //Where we started GoTo operation
uint32_t m_nGoToSlowDownMicrostep[2]; //Where we must start to deaccelerate
uint32_t m_nGoToTargetMicrostep[2]; //Where we must stop motors

TIM_HandleTypeDef* const TIMER_HANDLE[2] = {&TIMER_HANDLE_RA, &TIMER_HANDLE_DEC};
const uint32_t TIMER_CHANNEL[2] = {TIMER_CHANNEL_RA, TIMER_CHANNEL_DEC};
__IO uint32_t* TIMER_CCR[2] = {NULL, NULL};
GPIO_TypeDef* const DIR_GPIO[2] = {MOT_RA_DIR_GPIO_Port, MOT_DEC_DIR_GPIO_Port};
GPIO_TypeDef* const ENABLE_GPIO[2] = {MOT_RA_ENABLE_GPIO_Port, MOT_DEC_ENABLE_GPIO_Port};
const uint16_t DIR_GPIO_PIN[2] = {MOT_RA_DIR_Pin, MOT_DEC_DIR_Pin};
const uint16_t ENABLE_GPIO_PIN[2] = {MOT_RA_ENABLE_Pin, MOT_DEC_ENABLE_Pin};

//DMA buffer for timers
#define DMA_BUFFER_SIZE 128
uint8_t m_DMA_Buffer[DMA_BUFFER_SIZE];

En_Status StopMotorSlowly(En_MotorId nMotorId);
void ResetTimerDMA_Counter(En_MotorId nMotorId);
void SetMotorRate(En_MotorId nMotorId, uint32_t nRate);
En_Status StopMotorInstantly(En_MotorId nMotorId);

void InitLogic() {
	//timer instance is not filled during static init, that's why we need separate InitLogic() function
	TIMER_CCR[MI_RA] = &TIMER_CCR_RA;
	TIMER_CCR[MI_DEC] = &TIMER_CCR_DEC;
	EqReadConfig(&m_Config);
	//need to set DMA counters, otherwise it will be 1 step error on first start
	ResetTimerDMA_Counter(MI_RA);
	ResetTimerDMA_Counter(MI_DEC);
}

int32_t GetTimerDMA_Counter(En_MotorId nMotorId) {
	int32_t nDMA_Counter = m_nTimerDMA_Period[nMotorId] - TIMER_HANDLE[nMotorId]->hdma[TIM_DMA_ID_UPDATE]->Instance->CNDTR;
	return m_nMotorDirection[nMotorId] == DIR_FORWARD ? nDMA_Counter : -nDMA_Counter;
}

void ResetTimerDMA_Counter(En_MotorId nMotorId) {
	DMA_Channel_TypeDef* pDMA = TIMER_HANDLE[nMotorId]->hdma[TIM_DMA_ID_UPDATE]->Instance;
	if (pDMA->CCR & DMA_CCR_EN) { //if DMA channel is enabled
		pDMA->CCR &= ~DMA_CCR_EN;
		pDMA->CNDTR = m_nTimerDMA_Period[nMotorId];
		pDMA->CCR |= DMA_CCR_EN;
	} else {
		pDMA->CNDTR = m_nTimerDMA_Period[nMotorId];
	}
}

void CalcTimerDMA_Period(En_MotorId nMotorId, uint32_t nRate) {
	//we need interrupt on every microstep when nRate <= MaxAcceleration for precise GoTo
	uint32_t nPeriod = nRate / m_Config.m_AxisConfigs[nMotorId].m_nMotorMaxAcceleration;
	if (nPeriod == 0)
		nPeriod = 1;
	else if (nPeriod > DMA_BUFFER_SIZE)
		nPeriod = DMA_BUFFER_SIZE;
	m_nTimerDMA_Period[nMotorId] = (uint16_t)nPeriod;
}

void AdjustMicrostepCountAndResetTimerDMA_Counter(En_MotorId nMotorId, uint32_t nRate) {
	m_nMicrostepCount[nMotorId] += GetTimerDMA_Counter(nMotorId);
	CalcTimerDMA_Period(nMotorId, nRate);
	ResetTimerDMA_Counter(nMotorId);
}

template <typename T>
void SendResp(uint8_t* pRecvBuf, T* pResp) {
	uint8_t nSize = sizeof(*pResp);
	pRecvBuf[0] = nSize;
	memcpy(pRecvBuf + 1, pResp, nSize);
}

void SendResp(uint8_t* pRecvBuf, En_Status nStatus) {
	pRecvBuf[0] = sizeof(EqResp);
	((EqResp*)(pRecvBuf + 1))->m_nStatus = nStatus;
}

void DisableMotor(En_MotorId nMotorId) {
	m_lMotorDisableScheduled[nMotorId] = false;
	if (m_nMotorCurrentRate[nMotorId]) {
		m_lMotorDisableScheduled[nMotorId] = true;
		StopMotorSlowly(nMotorId); //recursion possible		
	} else if (m_lMotorEnabled[nMotorId]) {
		m_lMotorEnabled[nMotorId] = false;
		HAL_GPIO_WritePin(ENABLE_GPIO[nMotorId], ENABLE_GPIO_PIN[nMotorId], GPIO_PIN_RESET);
	}
}

void StartTimer(En_MotorId nMotorId) {
	TIM_HandleTypeDef *htim = TIMER_HANDLE[nMotorId];
	if (htim->State != HAL_TIM_STATE_READY) {
		return;
	}
	/* Change the htim state */
	htim->State = HAL_TIM_STATE_BUSY;
	/* Set the DMA Period elapsed callback */
	htim->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = TIM_DMADelayPulseCplt;
	/* Set the DMA error callback */
	htim->hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TIM_DMAError;	
	/* Enable the DMA channel */
	HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)htim->Instance->ARR, 
			(uint32_t)m_DMA_Buffer, m_nTimerDMA_Period[nMotorId]);
	/* Enable the TIM Update DMA request */
	__HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
	/* Enable the Capture compare channel */
	TIM_CCxChannelCmd(htim->Instance, TIMER_CHANNEL[nMotorId], TIM_CCx_ENABLE);
	/* Enable the Peripheral */
	__HAL_TIM_ENABLE(htim);
}

void StopTimer(En_MotorId nMotorId) {
	TIM_HandleTypeDef *htim = TIMER_HANDLE[nMotorId];
	/* Disable the TIM Update DMA request */
	__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
	/* Disable the Capture compare channel */
	TIM_CCxChannelCmd(htim->Instance, TIMER_CHANNEL[nMotorId], TIM_CCx_DISABLE);
	/* Disable the Peripheral */
	__HAL_TIM_DISABLE(htim);
	/* Set counter to 0 */
	__HAL_TIM_SET_COUNTER(htim, 0);
	/* Change the htim state */
	htim->State = HAL_TIM_STATE_READY;
}

En_Status StopMotorInstantly(En_MotorId nMotorId) {
	m_nMotorTargetRate[nMotorId] = 0;
	m_nMotorCurrentRate[nMotorId] = 0;
	m_nMotorChangeSpeedMicrostep[nMotorId] = 0;
	StopTimer(nMotorId);
	AdjustMicrostepCountAndResetTimerDMA_Counter(nMotorId, 0);
	
	if (m_lMotorDisableScheduled[nMotorId])
		DisableMotor(nMotorId);
	return STS_OK;
}

void SetMotorRate(En_MotorId nMotorId, uint32_t nRate) {
	uint32_t nPsc, nArr;
	if (nRate > m_Config.m_AxisConfigs[nMotorId].m_nMotorMaxRate) {
		nRate = m_Config.m_AxisConfigs[nMotorId].m_nMotorMaxRate;
	}
	if (nRate == 1) {
		nPsc = m_Config.m_AxisConfigs[nMotorId].m_nSiderealPsc;
		nArr = m_Config.m_AxisConfigs[nMotorId].m_nSiderealPeriod;
	} else {
		nPsc = 1;
		nArr = m_Config.m_AxisConfigs[nMotorId].m_nSiderealPsc *
				m_Config.m_AxisConfigs[nMotorId].m_nSiderealPeriod;
		nArr /= nRate;
		while (nArr > 0x10000) {
			nArr >>= 1;
			nPsc <<= 1;
		}
	}
	
	//we don't disable update event here because on high speed we can miss some steps
	//and exact rate isn't as important as steps accuracy in GoTo mode
	TIM_HandleTypeDef *pTimer = TIMER_HANDLE[nMotorId];
	pTimer->Instance->PSC = nPsc - 1;
	pTimer->Instance->ARR = nArr - 1;
	*TIMER_CCR[nMotorId] = nArr >> 1;	//PWM 50%
	AdjustMicrostepCountAndResetTimerDMA_Counter(nMotorId, nRate);
	m_nMotorCurrentRate[nMotorId] = nRate;
}

void AdjustSpeed(En_MotorId nMotorId) {
	uint32_t nRate;
	int32_t nDelta = m_nMotorTargetRate[nMotorId] - m_nMotorCurrentRate[nMotorId];
	uint16_t nAcceleration = m_Config.m_AxisConfigs[nMotorId].m_nMotorMaxAcceleration;
	if (nDelta > nAcceleration) {
		nRate = m_nMotorCurrentRate[nMotorId] + nAcceleration;
	} else if (nDelta < -nAcceleration) {
		nRate = m_nMotorCurrentRate[nMotorId] - nAcceleration;
	} else {
		nRate = m_nMotorTargetRate[nMotorId];
	}
	
	if (!nRate) {
		StopMotorInstantly(nMotorId);
		return;
	}
	SetMotorRate(nMotorId, nRate);
	
	m_nMotorLastAdjustedSpeedMicrostep[nMotorId] = m_nMicrostepCount[nMotorId];
	uint32_t nAccelerateStepsCount = nRate - nRate % m_nTimerDMA_Period[nMotorId];
	m_nMotorChangeSpeedMicrostep[nMotorId] = m_nMotorDirection[nMotorId] == DIR_FORWARD ?
					m_nMicrostepCount[nMotorId] + nAccelerateStepsCount :
					m_nMicrostepCount[nMotorId] - nAccelerateStepsCount;
}

En_Status StopMotorSlowly(En_MotorId nMotorId) {
	m_lGoToEnabled[nMotorId] = false;
	m_nMotorTargetRate[nMotorId] = 0;
	AdjustSpeed(nMotorId);
	return STS_OK;
}

En_Status StartMotor(En_MotorId nMotorId) {
	HAL_GPIO_WritePin(DIR_GPIO[nMotorId], DIR_GPIO_PIN[nMotorId],	
			m_nMotorDirection[nMotorId] == (DIR_FORWARD ^ m_Config.m_AxisConfigs[nMotorId].m_lReverse) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	CalcTimerDMA_Period(nMotorId, m_nMotorCurrentRate[nMotorId]);
	StartTimer(nMotorId);
	return STS_OK;
}

En_Status InitMotors(EqInitMotorsReq *pReq) {
	for (uint8_t nMotorId = MI_RA; nMotorId <= MI_DEC; nMotorId++) {
		if (m_nMotorCurrentRate[nMotorId])
			return nMotorId == MI_RA ? STS_RA_MOTOR_RUNNING : STS_DEC_MOTOR_RUNNING;
		
		m_nMicrostepCount[nMotorId] = (nMotorId == MI_RA ? pReq->m_nRaVal : pReq->m_nDecVal) << m_Config.m_AxisConfigs[nMotorId].m_nMicrostepsDivider;
		if (!m_lMotorEnabled[nMotorId]) {
			m_lMotorEnabled[nMotorId] = true;
			HAL_GPIO_WritePin(ENABLE_GPIO[nMotorId], ENABLE_GPIO_PIN[nMotorId], GPIO_PIN_SET);
		}
	}
	
	return STS_OK;
}

En_Status DeInitMotors() {
	DisableMotor(MI_RA);
	DisableMotor(MI_DEC);	
	return STS_OK;
}

En_Status GetMotorValues(EqGetMotorValuesReq *pReq, EqGetMotorValuesResp *pResp) {
	if (pReq->m_nMotorId == MI_RA) {
		EqGetEncoderValues(&pResp->m_nEncoderValueX, &pResp->m_nEncoderValueY);
	}
	
	pResp->m_nMicrostepCount = m_nMicrostepCount[pReq->m_nMotorId];
	if (m_nMotorCurrentRate[pReq->m_nMotorId]) {
		pResp->m_nMicrostepCount += GetTimerDMA_Counter((En_MotorId)pReq->m_nMotorId);
	}
	pResp->m_nMicrostepCount >>= m_Config.m_AxisConfigs[pReq->m_nMotorId].m_nMicrostepsDivider;
	return STS_OK;
}

En_Status SetMotorValues(EqSetMotorValuesReq *pReq) {
	if (m_nMotorCurrentRate[pReq->m_nMotorId])
		return STS_MOTOR_BUSY;
	if (!m_lMotorEnabled[pReq->m_nMotorId]) {
		return STS_MOTOR_NOT_INITIALIZED;
	}
		
	m_nMicrostepCount[pReq->m_nMotorId] = pReq->m_nMotorVal <<
			m_Config.m_AxisConfigs[pReq->m_nMotorId].m_nMicrostepsDivider;	
	return STS_OK;
}

En_Status Slew(EqSlewReq *pReq) {
	if (!m_lMotorEnabled[pReq->m_nMotorId])
		return STS_MOTOR_NOT_INITIALIZED;
	if (m_nMotorCurrentRate[pReq->m_nMotorId] &&
				m_nMotorDirection[pReq->m_nMotorId] != pReq->m_nDirection)
		return STS_MOTOR_BUSY;

	m_nMotorDirection[pReq->m_nMotorId] = pReq->m_nDirection;
	uint32_t nMaxRate = m_Config.m_AxisConfigs[pReq->m_nMotorId].m_nMotorMaxRate;
	m_nMotorTargetRate[pReq->m_nMotorId] = pReq->m_nRate <= nMaxRate ? pReq->m_nRate : nMaxRate;
	AdjustSpeed((En_MotorId)pReq->m_nMotorId);
	return StartMotor((En_MotorId)pReq->m_nMotorId);
}

En_Status GoTo(EqGoToReq *pReq) {
	if (!m_lMotorEnabled[pReq->m_nMotorId])
		return STS_MOTOR_NOT_INITIALIZED;
	if (m_nMotorCurrentRate[pReq->m_nMotorId])
		return STS_MOTOR_BUSY;
	if (pReq->m_nStepCount <= 0)
		return STS_INVALID_PARAMETERS;

	int32_t nDeltaSteps = pReq->m_nDirection == DIR_FORWARD ? 
					pReq->m_nStepCount : -pReq->m_nStepCount;
	m_nGoToStartMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId];
	int32_t nDelta = nDeltaSteps << m_Config.m_AxisConfigs[pReq->m_nMotorId].m_nMicrostepsDivider;
	m_nGoToTargetMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId] + nDelta;
	m_nGoToSlowDownMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId] + nDelta / 2;
	m_lGoToEnabled[pReq->m_nMotorId] = true;
	m_lGoToSlowDownAdjusted[pReq->m_nMotorId] = false;
	m_lGoToSlowedDown[pReq->m_nMotorId] = false;
	m_nMotorDirection[pReq->m_nMotorId] = pReq->m_nDirection;
	m_nMotorTargetRate[pReq->m_nMotorId] = m_Config.m_AxisConfigs[pReq->m_nMotorId].m_nMotorMaxRate;
	AdjustSpeed((En_MotorId)pReq->m_nMotorId);
	return StartMotor((En_MotorId)pReq->m_nMotorId);
}

En_Status StartTrack(EqStartTrackReq *pReq) {
	En_MotorId nMotorId = (En_MotorId)pReq->m_nMotorId;
	if (!m_lMotorEnabled[nMotorId])
		return STS_MOTOR_NOT_INITIALIZED;
	//next check is mandatory, otherwise calculated earlier DMA period can become wrong
	if (m_nMotorCurrentRate[nMotorId] > 1)
		return STS_MOTOR_BUSY;	
	
	TIM_HandleTypeDef *pTimer = TIMER_HANDLE[nMotorId];
	if (pReq->m_nFirstPrescaler > 1) {
		pTimer->Instance->ARR = pReq->m_nFirstPrescaler - 1;
		pTimer->Instance->PSC = pReq->m_nSecondPrescaler - 1;
	} else {
		pTimer->Instance->ARR = pReq->m_nSecondPrescaler - 1;
		pTimer->Instance->PSC = pReq->m_nFirstPrescaler - 1;
	}
	*TIMER_CCR[nMotorId] = pTimer->Instance->ARR >> 1; //PWM 50%
	
	m_nMotorDirection[nMotorId] = pReq->m_nDirection;
	m_nMotorTargetRate[nMotorId] = 1;
	En_Status nStatus = STS_OK;
	if (!m_nMotorCurrentRate[nMotorId]) {
		m_nMotorCurrentRate[nMotorId] = 1;
		nStatus = StartMotor(nMotorId);
	}
	m_nMotorCurrentRate[nMotorId] = 1;
	return nStatus;
}

//====================================================

void EqProcessReceive(uint8_t* pRecvBuf) {
	uint8_t nReqSize = pRecvBuf[0];
	switch (((EqReq*)(pRecvBuf + 1))->m_nCmd) {
		case CMD_INIT_MOTORS: {
			if (nReqSize != sizeof(EqInitMotorsReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, InitMotors((EqInitMotorsReq*)(pRecvBuf + 1)));
		}
		case CMD_DEINIT_MOTORS: {
			return SendResp(pRecvBuf, DeInitMotors());
		}
		case CMD_GET_MOTOR_STATUS: {
			if (nReqSize != sizeof(EqGetMotorStatusReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqGetMotorStatusReq *pReq = (EqGetMotorStatusReq*)(pRecvBuf + 1);
			EqGetMotorStatusResp Resp(
							m_lMotorEnabled[pReq->m_nMotorId],
							m_nMotorCurrentRate[pReq->m_nMotorId] != 0,
							(En_Direction)m_nMotorDirection[pReq->m_nMotorId]);
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_GET_MOTOR_VALUES: {
			if (nReqSize != sizeof(EqGetMotorValuesReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqGetMotorValuesResp Resp;
			Resp.m_nStatus = GetMotorValues(
														(EqGetMotorValuesReq*)(pRecvBuf + 1),
														&Resp);
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_SET_MOTOR_VALUES: {
			if (nReqSize != sizeof(EqSetMotorValuesReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, SetMotorValues((EqSetMotorValuesReq*)(pRecvBuf + 1)));
		}
		case CMD_STOP_MOTOR: {
			if (nReqSize != sizeof(EqStopMotorReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, StopMotorSlowly(
				(En_MotorId)((EqStopMotorReq*)(pRecvBuf + 1))->m_nMotorId));
		}
		case CMD_SLEW: {
			if (nReqSize != sizeof(EqSlewReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, Slew((EqSlewReq*)(pRecvBuf + 1)));
		}
		case CMD_GOTO: {
			if (nReqSize != sizeof(EqGoToReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, GoTo((EqGoToReq*)(pRecvBuf + 1)));
		}
		case CMD_START_TRACK: {
			if (nReqSize != sizeof(EqStartTrackReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, StartTrack((EqStartTrackReq*)(pRecvBuf + 1)));
		}
		case CMD_READ_CONFIG: {
			if (nReqSize != sizeof(EqReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqReadConfigResp Resp;
			Resp.m_Config = m_Config;
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_WRITE_CONFIG: {
			if (nReqSize != sizeof(EqWriteConfigReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			Config& config = ((EqWriteConfigReq*)(pRecvBuf + 1))->m_Config;
			En_Status status = (En_Status)EqWriteConfig(&config);
			if (status == STS_OK) {
				EqReadConfig(&m_Config);
			}
			return SendResp(pRecvBuf, status);
		}
		case CMD_READ_ENCODER_CORRECTION: {
			if (nReqSize != sizeof(EqReadEncoderCorrectionReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqReadEncoderCorrectionResp Resp;
			EqReadEncoderCorrection(((EqReadEncoderCorrectionReq*)(pRecvBuf + 1))->m_nPageNumber, Resp.m_Data);
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_WRITE_ENCODER_CORRECTION: {
			if (nReqSize != sizeof(EqWriteEncoderCorrectionReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqWriteEncoderCorrectionReq* pReq = (EqWriteEncoderCorrectionReq*)(pRecvBuf + 1);
			En_Status status = (En_Status)EqWriteEncoderCorrection(pReq->m_nPageNumber, pReq->m_Data);
			return SendResp(pRecvBuf, status);
		}
		case CMD_CLEAR_ENCODER_CORRECTION: {
			return SendResp(pRecvBuf, (En_Status)EqClearEncoderCorrection());
		}
		case CMD_READ_PEC: {
			if (nReqSize != sizeof(EqReadPEC_Req))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqReadPEC_Resp Resp;
			EqReadPEC(((EqReadPEC_Req*)(pRecvBuf + 1))->m_nPageNumber, Resp.m_Data);
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_WRITE_PEC: {
			if (nReqSize != sizeof(EqWritePEC_Req))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			EqWritePEC_Req* pReq = (EqWritePEC_Req*)(pRecvBuf + 1);
			En_Status status = (En_Status)EqWritePEC(pReq->m_nPageNumber, pReq->m_Data);
			return SendResp(pRecvBuf, status);
		}
		case CMD_CLEAR_PEC: {
			return SendResp(pRecvBuf, (En_Status)EqClearPEC());
		}
		default:
			return SendResp(pRecvBuf, STS_UNKNOWN_CMD);
	}
}

bool CheckMicrostepsEquals(En_MotorId nMotorId, uint32_t nCheckMicrosteps) {
	if (m_nMotorDirection[nMotorId] == DIR_FORWARD) {
		return m_nMicrostepCount[nMotorId] >= nCheckMicrosteps;
	}
	return m_nMicrostepCount[nMotorId] <= nCheckMicrosteps;
}

//Timers interrupt handler (DMA update event in our case)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	En_MotorId nMotorId;
  if (htim == &TIMER_HANDLE_RA) {
		nMotorId = MI_RA;
	} else if (htim == &TIMER_HANDLE_DEC) {
		nMotorId = MI_DEC;
	} else {
		return;
	}
	
	if (m_nMotorDirection[nMotorId] == DIR_FORWARD) {
		m_nMicrostepCount[nMotorId] += m_nTimerDMA_Period[nMotorId];
	} else {
		m_nMicrostepCount[nMotorId] -= m_nTimerDMA_Period[nMotorId];
	}
	
	if (m_lGoToEnabled[nMotorId]) {
		//check if we reached target
		if (CheckMicrostepsEquals(nMotorId, m_nGoToTargetMicrostep[nMotorId])) {
			StopMotorSlowly(nMotorId);
			return;
		}
		//check if we reached motor slowdown point
		if (!m_lGoToSlowedDown[nMotorId] && 
				CheckMicrostepsEquals(nMotorId, m_nGoToSlowDownMicrostep[nMotorId])) {
			if (!m_lGoToSlowDownAdjusted[nMotorId]) {
				m_lGoToSlowDownAdjusted[nMotorId] = true;
				//how many steps we need to do before slowdown
				int32_t nDelta = m_nGoToSlowDownMicrostep[nMotorId] * 2 - m_nMotorLastAdjustedSpeedMicrostep[nMotorId] - m_nMicrostepCount[nMotorId];
				nDelta -= nDelta % m_nTimerDMA_Period[nMotorId];
				if (nDelta) {
					m_nGoToSlowDownMicrostep[nMotorId] = m_nMicrostepCount[nMotorId] + nDelta;
					m_nMotorChangeSpeedMicrostep[nMotorId] = m_nGoToSlowDownMicrostep[nMotorId];
					return;
				}
			}
			m_lGoToSlowedDown[nMotorId] = true;
			m_nMotorTargetRate[nMotorId] = m_Config.m_AxisConfigs[nMotorId].m_nMotorMaxAcceleration;
			AdjustSpeed(nMotorId);
			return;
		}
		//if we reached full speed earlier than slowdown point, then we must adjust that point
		if (!m_lGoToSlowDownAdjusted[nMotorId] &&
				m_nMotorCurrentRate[nMotorId] == m_nMotorTargetRate[nMotorId]) {
			//How many steps acceleration took
			int32_t nAccelerationStepsCount = 
					m_nMicrostepCount[nMotorId] - m_nGoToStartMicrostep[nMotorId]; 
			m_nGoToSlowDownMicrostep[nMotorId] = 
					m_nGoToTargetMicrostep[nMotorId] - nAccelerationStepsCount;
			m_lGoToSlowDownAdjusted[nMotorId] = true;
		}
	}
	
	//accelerate / decelerate
	if (m_nMotorCurrentRate[nMotorId] != m_nMotorTargetRate[nMotorId] &&
			CheckMicrostepsEquals(nMotorId, m_nMotorChangeSpeedMicrostep[nMotorId])) {
		AdjustSpeed(nMotorId);
	}
}

//called once every 1ms
//void HAL_SYSTICK_Callback(void) {
//	for (uint8_t nMotorId = MI_RA; nMotorId <= MI_DEC; nMotorId++) {
//		if (m_lMotorStartScheduled[nMotorId] && m_nScheduledMotorStartTicks[nMotorId] == HAL_GetTick()) {
//			StartTimer((En_MotorId)nMotorId);
//			m_lMotorStartScheduled[nMotorId] = false;
//			m_nScheduledMotorStartTicks[nMotorId] = 0;
//		}
//	}
//}
