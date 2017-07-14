#include <mxconstants.h>
#include <gpio.h>
#include <string.h>
#include <tim.h>
#include <Protocol.h>
#include "Encoder.h"
#include "Config.h"
#include "Logic.h"

using namespace EQ;

uint32_t m_nMicrostepCount[2] = {0, 0};
bool m_lMotorEnabled[2] = {false, false};
bool m_lMotorDisableScheduled[2] = {false, false};
uint8_t m_nMotorDirection[2] = {DIR_FORWARD, DIR_FORWARD};
uint32_t m_nMotorTargetRate[2] = {1, 1}; //1 = Sidereal
uint32_t m_nMotorCurrentRate[2] = {0, 0};
uint16_t m_nMotorMaxRate[2] = {800, 200};
uint16_t m_nMotorMaxAcceleration[2] = {5, 2}; //Sidereal rates per 12.5 ms
uint32_t m_nMotorChangeSpeedMicrostep[2] = {0, 0};

uint16_t m_nEmergencyStopAccelerationMultiplier = 3;
bool m_lDecMotorEmergencyStop = false;

bool m_lGoToEnabled[2] = {false, false}; //If GoTo command is in process
bool m_lGoToSlowDownAdjusted[2] = {false, false}; //If we moved slowdown point due to reaching full speed
uint32_t m_nGoToStartMicrostep[2]; //Where we started GoTo operation
uint32_t m_nGoToSlowDownMicrostep[2]; //Where we must start to deaccelerate
uint32_t m_nGoToTargetMicrostep[2]; //Where we must stop motors

En_Status StopMotorSlowly(En_MotorId nMotorId);

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
		HAL_GPIO_WritePin(
				nMotorId == MI_RA ? MOT_RA_ENABLE_GPIO_Port : MOT_DEC_ENABLE_GPIO_Port, 
				nMotorId == MI_RA ? MOT_RA_ENABLE_Pin : MOT_DEC_ENABLE_Pin, 
				GPIO_PIN_RESET);
	}
}

En_Status StopMotorInstantly(En_MotorId nMotorId) {
	m_nMotorTargetRate[nMotorId] = 0;
	m_nMotorCurrentRate[nMotorId] = 0;
	m_nMotorChangeSpeedMicrostep[nMotorId] = 0;
	if (nMotorId == MI_RA) {
		HAL_TIM_OC_Stop_IT(&TIMER_HANDLE_RA, TIMER_CHANNEL_RA);
		__HAL_TIM_SetCounter(&TIMER_HANDLE_RA, 0);
	} else {
		HAL_TIM_OC_Stop_IT(&TIMER_HANDLE_DEC, TIMER_CHANNEL_DEC);
		__HAL_TIM_SetCounter(&TIMER_HANDLE_DEC, 0);
		m_lDecMotorEmergencyStop = false;
	}
	
	if (m_lMotorDisableScheduled[nMotorId])
		DisableMotor(nMotorId);
	return STS_OK;
}

void SetMotorRate(En_MotorId nMotorId, uint32_t nRate) {
	uint32_t nPsc, nArr;
	if (nRate > m_nMotorMaxRate[nMotorId]) {
		nRate = m_nMotorMaxRate[nMotorId];
	}
	if (nRate == 1) {
		nPsc = nMotorId == MI_RA ? MOT_RA_PSC_SIDEREAL : MOT_DEC_PSC_SIDEREAL;
		nArr = nMotorId == MI_RA ? MOT_RA_PERIOD_SIDEREAL : MOT_DEC_PERIOD_SIDEREAL;
	} else {
		nPsc = 1;
		nArr = nMotorId == MI_RA ?
			MOT_RA_PSC_SIDEREAL * MOT_RA_PERIOD_SIDEREAL :
			MOT_DEC_PSC_SIDEREAL * MOT_DEC_PERIOD_SIDEREAL;
		nArr /= nRate;
		while (nArr > 0x10000) {
			nArr >>= 1;
			nPsc <<= 1;
		}
	}
	
	if (nMotorId == MI_RA) {
		TIMER_HANDLE_RA.Instance->PSC = nPsc - 1;
		TIMER_HANDLE_RA.Instance->ARR = nArr - 1;
	} else {
		TIMER_HANDLE_DEC.Instance->PSC = nPsc - 1;
		TIMER_HANDLE_DEC.Instance->ARR = nArr - 1;
	}
	
	m_nMotorCurrentRate[nMotorId] = nRate;
}

void AdjustSpeed(En_MotorId nMotorId) {
	uint32_t nRate;
	int32_t nDelta = m_nMotorTargetRate[nMotorId] - m_nMotorCurrentRate[nMotorId];
	uint16_t nAcceleration = nMotorId == MI_DEC && m_lDecMotorEmergencyStop ? 
			m_nEmergencyStopAccelerationMultiplier * m_nMotorMaxAcceleration[MI_DEC] :
			m_nMotorMaxAcceleration[nMotorId];
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
	
	uint32_t nAccelerateStepsCount = nRate;
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

bool CheckMotorLimits(En_MotorId nMotorId, En_Direction nDirection) {
	if (nMotorId == MI_DEC) {
		if (nDirection == DIR_FORWARD && 
				HAL_GPIO_ReadPin(DEC_LIMIT_FORWARD_GPIO_Port, DEC_LIMIT_FORWARD_Pin) == GPIO_PIN_RESET)
			return false;
		if (nDirection == DIR_REVERSE && 
				HAL_GPIO_ReadPin(DEC_LIMIT_REVERSE_GPIO_Port, DEC_LIMIT_REVERSE_Pin) == GPIO_PIN_RESET)
			return false;
	}
	return true;
}

En_Status StartMotor(En_MotorId nMotorId) {
	if (!CheckMotorLimits(nMotorId, (En_Direction)m_nMotorDirection[nMotorId])) {
		StopMotorSlowly(nMotorId); //need to erase all motor params
		return STS_MOTOR_LIMIT_REACHED;
	}
	if (nMotorId == MI_RA) {
		HAL_GPIO_WritePin(MOT_RA_DIR_GPIO_Port, MOT_RA_DIR_Pin, 
				m_nMotorDirection[MI_RA] == DIR_FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_TIM_OC_Start_IT(&TIMER_HANDLE_RA, TIMER_CHANNEL_RA);
	} else {
		HAL_GPIO_WritePin(MOT_DEC_DIR_GPIO_Port, MOT_DEC_DIR_Pin, 
				m_nMotorDirection[MI_DEC] == DIR_FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_TIM_OC_Start_IT(&TIMER_HANDLE_DEC, TIMER_CHANNEL_DEC);
	}
	return STS_OK;
}

En_Status InitMotors(EqInitMotorsReq *pReq) {
	if (m_nMotorCurrentRate[MI_RA])
		return STS_RA_MOTOR_RUNNING;
	if (m_nMotorCurrentRate[MI_DEC])
		return STS_DEC_MOTOR_RUNNING;
	
	m_nMicrostepCount[MI_RA] = pReq->m_nRaVal << 1;
	if (!m_lMotorEnabled[MI_RA]) {
		m_lMotorEnabled[MI_RA] = true;
		HAL_GPIO_WritePin(MOT_RA_ENABLE_GPIO_Port, MOT_RA_ENABLE_Pin, GPIO_PIN_SET);
	}
	
	m_nMicrostepCount[MI_DEC] = pReq->m_nDecVal << 1;
	if (!m_lMotorEnabled[MI_DEC]) {
		m_lMotorEnabled[MI_DEC] = true;
		HAL_GPIO_WritePin(MOT_DEC_ENABLE_GPIO_Port, MOT_DEC_ENABLE_Pin, GPIO_PIN_SET);
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
	pResp->m_nMicrostepCount = m_nMicrostepCount[pReq->m_nMotorId] >> 1;
	return STS_OK;
}

En_Status SetMotorValues(EqSetMotorValuesReq *pReq) {
	if (m_nMotorCurrentRate[pReq->m_nMotorId])
		return STS_MOTOR_BUSY;
	if (!m_lMotorEnabled[pReq->m_nMotorId]) {
		return STS_MOTOR_NOT_INITIALIZED;
	}
		
	m_nMicrostepCount[pReq->m_nMotorId] = pReq->m_nMotorVal << 1;	
	return STS_OK;
}

En_Status Slew(EqSlewReq *pReq) {
	if (!m_lMotorEnabled[pReq->m_nMotorId])
		return STS_MOTOR_NOT_INITIALIZED;
	if (m_nMotorCurrentRate[pReq->m_nMotorId] &&
				m_nMotorDirection[pReq->m_nMotorId] != pReq->m_nDirection)
		return STS_MOTOR_BUSY;

	m_nMotorDirection[pReq->m_nMotorId] = pReq->m_nDirection;
	m_nMotorTargetRate[pReq->m_nMotorId] = pReq->m_nRate;
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

	uint32_t nDeltaSteps = pReq->m_nDirection == DIR_FORWARD ? 
					pReq->m_nStepCount : -pReq->m_nStepCount;
	m_nGoToStartMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId];
	m_nGoToTargetMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId] + (nDeltaSteps << 1);
	m_nGoToSlowDownMicrostep[pReq->m_nMotorId] = m_nMicrostepCount[pReq->m_nMotorId] + nDeltaSteps;
	m_lGoToEnabled[pReq->m_nMotorId] = true;
	m_lGoToSlowDownAdjusted[pReq->m_nMotorId] = false;
	m_nMotorDirection[pReq->m_nMotorId] = pReq->m_nDirection;
	m_nMotorTargetRate[pReq->m_nMotorId] = m_nMotorMaxRate[pReq->m_nMotorId];
	AdjustSpeed((En_MotorId)pReq->m_nMotorId);
	return StartMotor((En_MotorId)pReq->m_nMotorId);
}

En_Status StartTrack(EqStartTrackReq *pReq) {
	En_MotorId nMotorId = (En_MotorId)pReq->m_nMotorId;
	if (!m_lMotorEnabled[nMotorId])
		return STS_MOTOR_NOT_INITIALIZED;
	if (m_nMotorCurrentRate[nMotorId] > m_nMotorMaxAcceleration[nMotorId])
		return STS_MOTOR_BUSY;
	if (m_nMotorCurrentRate[nMotorId] <= m_nMotorMaxAcceleration[nMotorId])
		StopMotorInstantly(nMotorId);

	m_nMotorDirection[nMotorId] = pReq->m_nDirection;
	m_nMotorTargetRate[nMotorId] = 1;
	m_nMotorCurrentRate[nMotorId] = 1;
	
	TIM_HandleTypeDef *pTimer;
	if (nMotorId == MI_RA) {
		pTimer = &TIMER_HANDLE_RA;
	} else {
		pTimer = &TIMER_HANDLE_DEC;
	}
	pTimer->Instance->ARR = pReq->m_nFirstPrescaler > 1 ? pReq->m_nFirstPrescaler - 1 : pReq->m_nSecondPrescaler - 1;
	pTimer->Instance->PSC = pReq->m_nFirstPrescaler > 1 ? pReq->m_nSecondPrescaler - 1 : pReq->m_nFirstPrescaler - 1;
	return StartMotor(nMotorId);
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
			if (nReqSize != sizeof(EqDeInitMotorsReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
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
			EqReadConfig(&Resp.m_Config);
			return SendResp(pRecvBuf, &Resp);
		}
		case CMD_WRITE_CONFIG: {
			if (nReqSize != sizeof(EqWriteConfigReq))
				return SendResp(pRecvBuf, STS_WRONG_CMD_SIZE);
			
			return SendResp(pRecvBuf, (En_Status)EqWriteConfig(&((EqWriteConfigReq*)(pRecvBuf + 1))->m_Config));
		}
		default:
			return SendResp(pRecvBuf, STS_UNKNOWN_CMD);
	}
}

//Timers output compare interrupt handler
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	En_MotorId nMotorId;
  if (htim == &TIMER_HANDLE_DEC && htim->Channel == TIMER_ACTIVE_CHANNEL_DEC) {
		nMotorId = MI_DEC;
	} else if (htim == &TIMER_HANDLE_RA && htim->Channel == TIMER_ACTIVE_CHANNEL_RA) {
		nMotorId = MI_RA;
	} else {
		return;
	}
	
	if (m_nMotorDirection[nMotorId] == DIR_FORWARD) {
		m_nMicrostepCount[nMotorId]++;
	} else {
		m_nMicrostepCount[nMotorId]--;
	}
	
	if (m_lGoToEnabled[nMotorId]) {
		if (m_nMicrostepCount[nMotorId] == m_nGoToTargetMicrostep[nMotorId]) {
			StopMotorSlowly(nMotorId);
			return;
		}
		if (m_nMicrostepCount[nMotorId] == m_nGoToSlowDownMicrostep[nMotorId]) {
			m_lGoToSlowDownAdjusted[nMotorId] = true;
			m_nMotorTargetRate[nMotorId] = m_nMotorMaxAcceleration[nMotorId];
			AdjustSpeed(nMotorId);
			return;
		}
		if (!m_lGoToSlowDownAdjusted[nMotorId] && 
				m_nMotorCurrentRate[nMotorId] == m_nMotorTargetRate[nMotorId]) {
			//How many steps acceleration took
			uint32_t nAccelerationStepsCount = 
					m_nMicrostepCount[nMotorId] - m_nGoToStartMicrostep[nMotorId]; 
			m_nGoToSlowDownMicrostep[nMotorId] = 
					m_nGoToTargetMicrostep[nMotorId] - nAccelerationStepsCount;
			m_lGoToSlowDownAdjusted[nMotorId] = true;
		}
	}
	
	if (m_nMicrostepCount[nMotorId] != m_nMotorChangeSpeedMicrostep[nMotorId] ||
				m_nMotorCurrentRate[nMotorId] == m_nMotorTargetRate[nMotorId])
		return;
	
	AdjustSpeed(nMotorId);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if (!m_lDecMotorEmergencyStop && m_nMotorCurrentRate[MI_DEC] && 
				((GPIO_Pin == DEC_LIMIT_FORWARD_Pin && m_nMotorDirection[MI_DEC] == DIR_FORWARD) ||
				(GPIO_Pin == DEC_LIMIT_REVERSE_Pin && m_nMotorDirection[MI_DEC] == DIR_REVERSE))) {
			m_lDecMotorEmergencyStop = true;
			StopMotorSlowly(MI_DEC);
		}
}

//void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc) {
//	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//}
