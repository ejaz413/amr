//////////////////////////////////////////////////////////////////////////
//
//	File:			FAS_EziMOTIONPlusE.h
//	Last Modified Date:	24.08.09
//	Last Modified Ver:	1.0.2
//
//////////////////////////////////////////////////////////////////////////

#pragma once

#include <codecvt>
#include <locale>

#include "ReturnCodes_Define.h"
#include "MOTION_DEFINE.h"

extern "C"
{
	//------------------------------------------------------------------------------
	//			Connection Functions
	//------------------------------------------------------------------------------
	//class PE
	namespace PE
	{
		int 	FAS_Connect(unsigned char sb1, unsigned char sb2, unsigned char sb3, unsigned char sb4, int iBdID);		// UDP Protocol
		int 	FAS_ConnectTCP(unsigned char sb1, unsigned char sb2, unsigned char sb3, unsigned char sb4, int iBdID);	// TCP Protocol

		int 	FAS_IsBdIDExist(int iBdID, unsigned char* sb1, unsigned char* sb2, unsigned char* sb3, unsigned char* sb4);
		int 	FAS_IsIPAddressExist(unsigned char sb1, unsigned char sb2, unsigned char sb3, unsigned char sb4, int* iBdID);

		int 	FAS_Reconnect(int iBdID);
		void 	FAS_SetAutoReconnect(int bSET);

		void 	FAS_Close(int iBdID);

		int 	FAS_IsSlaveExist(int iBdID);

		//------------------------------------------------------------------------------
		//			Ethernet Address Functions
		//------------------------------------------------------------------------------
		int 	FAS_GetEthernetAddr(int iBdID, unsigned int* gateway, unsigned int* subnet, unsigned int* ip);
		int 	FAS_SetEthernetAddr(int iBdID, unsigned int gateway, unsigned int subnet, unsigned int ip);

		int 	FAS_GetMACAddress(int iBdID, unsigned long long* MACAddress);

		int 	FAS_EnableTwoTCPPort(int iBdID, int bEnable);

		//------------------------------------------------------------------------------
		//			Log Functions
		//------------------------------------------------------------------------------
		void 	FAS_EnableLog(int bEnable);
		void 	FAS_SetLogLevel(enum LOG_LEVEL level);
		int 	FAS_SetLogPath(const wchar_t* lpPath);

		void 	FAS_PrintCustomLog(int iBdID, enum LOG_LEVEL level, const wchar_t* lpszMsg);

		//------------------------------------------------------------------------------
		//			Info Functions
		//------------------------------------------------------------------------------
		int 	FAS_GetSlaveInfo(int iBdID, unsigned char* pType, char* lpBuff, int nBuffSize);
		int 	FAS_GetMotorInfo(int iBdID, unsigned char* pType, char* lpBuff, int nBuffSize);
		int 	FAS_GetSlaveInfoEx(int iBdID, DRIVE_INFO* lpDriveInfo);

		//------------------------------------------------------------------------------
		//			Parameter Functions
		//------------------------------------------------------------------------------
		int 	FAS_SaveAllParameters(int iBdID);
		int 	FAS_SetParameter(int iBdID, unsigned char iParamNo, int lParamValue);
		int 	FAS_GetParameter(int iBdID, unsigned char iParamNo, int* lParamValue);
		int 	FAS_GetROMParameter(int iBdID, unsigned char iParamNo, int* lRomParam);

		//------------------------------------------------------------------------------
		//			IO Functions
		//------------------------------------------------------------------------------
		int 	FAS_SetIOInput(int iBdID, unsigned int dwIOSETMask, unsigned int dwIOCLRMask);
		int 	FAS_GetIOInput(int iBdID, unsigned int* dwIOInput);

		int 	FAS_SetIOOutput(int iBdID, unsigned int dwIOSETMask, unsigned int dwIOCLRMask);
		int 	FAS_GetIOOutput(int iBdID, unsigned int* dwIOOutput);

		int 	FAS_GetIOAssignMap(int iBdID, unsigned char iIOPinNo, unsigned int* dwIOLogicMask, unsigned char* bLevel);
		int 	FAS_SetIOAssignMap(int iBdID, unsigned char iIOPinNo, unsigned int dwIOLogicMask, unsigned char bLevel);

		int 	FAS_IOAssignMapReadROM(int iBdID);

		//------------------------------------------------------------------------------
		//			Servo Driver Control Functions
		//------------------------------------------------------------------------------
		int 	FAS_ServoEnable(int iBdID, int bOnOff);
		int 	FAS_ServoAlarmReset(int iBdID);
		int 	FAS_StepAlarmReset(int iBdID, int bReset);
		int 	FAS_BrakeSet(int iBdID, int bSet, int* nResult);

		//------------------------------------------------------------------------------
		//			Read Status and Position
		//------------------------------------------------------------------------------
		int 	FAS_GetAxisStatus(int iBdID, unsigned int* dwAxisStatus);
		int 	FAS_GetIOAxisStatus(int iBdID, unsigned int* dwInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus);
		int 	FAS_GetMotionStatus(int iBdID, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);
		int 	FAS_GetAllStatus(int iBdID, unsigned int* dwInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);
		int 	FAS_GetAllStatusEx(int iBdID, unsigned char* pTypes, int* pDatas);

		int 	FAS_SetCommandPos(int iBdID, int lCmdPos);
		int 	FAS_SetActualPos(int iBdID, int lActPos);
		int 	FAS_ClearPosition(int iBdID);
		int 	FAS_GetCommandPos(int iBdID, int* lCmdPos);
		int 	FAS_GetActualPos(int iBdID, int* lActPos);
		int 	FAS_GetPosError(int iBdID, int* lPosErr);
		int 	FAS_GetActualVel(int iBdID, int* lActVel);

		int 	FAS_GetAlarmType(int iBdID, unsigned char* nAlarmType);

		//------------------------------------------------------------------
		//			Motion Functions.
		//------------------------------------------------------------------
		int 	FAS_MoveStop(int iBdID);
		int 	FAS_EmergencyStop(int iBdID);

		int 	FAS_MovePause(int iBdID, int bPause);

		int 	FAS_MoveOriginSingleAxis(int iBdID);
		int 	FAS_MoveSingleAxisAbsPos(int iBdID, int lAbsPos, unsigned int lVelocity);
		int 	FAS_MoveSingleAxisIncPos(int iBdID, int lIncPos, unsigned int lVelocity);
		int 	FAS_MoveToLimit(int iBdID, unsigned int lVelocity, int iLimitDir);
		int 	FAS_MoveVelocity(int iBdID, unsigned int lVelocity, int iVelDir);

		int 	FAS_PositionAbsOverride(int iBdID, int lOverridePos);
		int 	FAS_PositionIncOverride(int iBdID, int lOverridePos);
		int 	FAS_VelocityOverride(int iBdID, unsigned int lVelocity);

		int 	FAS_MoveLinearAbsPos(unsigned char nNoOfBds, int* iBdID, int* lplAbsPos, unsigned int lFeedrate, unsigned short wAccelTime);
		int 	FAS_MoveLinearIncPos(unsigned char nNoOfBds, int* iBdID, int* lplIncPos, unsigned int lFeedrate, unsigned short wAccelTime);

		int 	FAS_MoveLinearAbsPos2(unsigned char nNoOfBds, int* iBdID, int* lplAbsPos, unsigned int lFeedrate, unsigned short wAccelTime);
		int 	FAS_MoveLinearIncPos2(unsigned char nNoOfBds, int* iBdID, int* lplIncPos, unsigned int lFeedrate, unsigned short wAccelTime);

		int 	FAS_MoveCircleAbsPos1(unsigned char nNoOfBds, int* iBdID, int* lplCirEndAbs, int* lplCirCenterAbs, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
		int 	FAS_MoveCircleIncPos1(unsigned char nNoOfBds, int* iBdID, int* lplCirEndInc, int* lplCirCenterInc, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
		int 	FAS_MoveCircleAbsPos2(unsigned char nNoOfBds, int* iBdID, int* lplCirEndAbs, unsigned int lRadius, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
		int 	FAS_MoveCircleIncPos2(unsigned char nNoOfBds, int* iBdID, int* lplCirEndInc, unsigned int lRadius, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
		int 	FAS_MoveCircleAbsPos3(unsigned char nNoOfBds, int* iBdID, int* lplCirCenterAbs, unsigned int nAngle, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
		int 	FAS_MoveCircleIncPos3(unsigned char nNoOfBds, int* iBdID, int* lplCirCenterInc, unsigned int nAngle, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);

		int 	FAS_TriggerOutput_RunA(int iBdID, int bStartTrigger, int lStartPos, unsigned int dwPeriod, unsigned int dwPulseTime);
		int 	FAS_TriggerOutput_Status(int iBdID, unsigned char* bTriggerStatus);

		int 	FAS_SetTriggerOutputEx(int iBdID, unsigned char nOutputNo, unsigned char bRun, unsigned short wOnTime, unsigned char nTriggerCount, int* arrTriggerPosition);
		int 	FAS_GetTriggerOutputEx(int iBdID, unsigned char nOutputNo, unsigned char* bRun, unsigned short* wOnTime, unsigned char* nTriggerCount, int* arrTriggerPosition);

		int 	FAS_MovePush(int iBdID, unsigned int dwStartSpd, unsigned int dwMoveSpd, int lPosition, unsigned short wAccel, unsigned short wDecel, unsigned short wPushRate, unsigned int dwPushSpd, int lEndPosition, unsigned short wPushMode);
		int 	FAS_GetPushStatus(int iBdID, unsigned char* nPushStatus);

		//------------------------------------------------------------------
		//			Ex-Motion Functions.
		//------------------------------------------------------------------
		int 	FAS_MoveSingleAxisAbsPosEx(int iBdID, int lAbsPos, unsigned int lVelocity, MOTION_OPTION_EX* lpExOption);
		int 	FAS_MoveSingleAxisIncPosEx(int iBdID, int lIncPos, unsigned int lVelocity, MOTION_OPTION_EX* lpExOption);
		int 	FAS_MoveVelocityEx(int iBdID, unsigned int lVelocity, int iVelDir, VELOCITY_OPTION_EX* lpExOption);

		//------------------------------------------------------------------
		//			Position Table Functions.
		//------------------------------------------------------------------
		int 	FAS_PosTableReadItem(int iBdID, unsigned short wItemNo, LPITEM_NODE lpItem);
		int 	FAS_PosTableWriteItem(int iBdID, unsigned short wItemNo, LPITEM_NODE lpItem);
		int 	FAS_PosTableWriteROM(int iBdID);
		int 	FAS_PosTableReadROM(int iBdID);
		int 	FAS_PosTableRunItem(int iBdID, unsigned short wItemNo);

		int 	FAS_PosTableReadOneItem(int iBdID, unsigned short wItemNo, unsigned short wOffset, int* lPosItemVal);
		int 	FAS_PosTableWriteOneItem(int iBdID, unsigned short wItemNo, unsigned short wOffset, int lPosItemVal);

		int 	FAS_PosTableSingleRunItem(int iBdID, int bNextMove, unsigned short wItemNo);

		//------------------------------------------------------------------
		//			Gap Control Functions.
		//------------------------------------------------------------------
		int 	FAS_GapControlEnable(int iBdID, unsigned short wItemNo, int lGapCompSpeed, int lGapAccTime, int lGapDecTime, int lGapStartSpeed);
		int 	FAS_GapControlDisable(int iBdID);
		int 	FAS_IsGapControlEnable(int iBdID, int* bIsEnable, unsigned short* wCurrentItemNo);

		int 	FAS_GapControlGetADCValue(int iBdID, int* lADCValue);
		int 	FAS_GapOneResultMonitor(int iBdID, unsigned char* bUpdated, int* iIndex, int* lGapValue, int* lCmdPos, int* lActPos, int* lCompValue, int* lReserved);

		//------------------------------------------------------------------
		//			Alarm Type History Functions.
		//------------------------------------------------------------------
		int 	FAS_GetAlarmLogs(int iBdID, ALARM_LOG* pAlarmLog);
		int 	FAS_ResetAlarmLogs(int iBdID);

		//------------------------------------------------------------------
		//			I/O Module Functions.
		//------------------------------------------------------------------
		int 	FAS_GetInput(int iBdID, unsigned int* uInput, unsigned int* uLatch);

		int 	FAS_ClearLatch(int iBdID, unsigned int uLatchMask);
		int 	FAS_GetLatchCount(int iBdID, unsigned char iInputNo, unsigned int* uCount);
		int 	FAS_GetLatchCountAll(int iBdID, unsigned int* ppuAllCount);
		int 	FAS_GetLatchCountAll32(int iBdID, unsigned int* ppuAllCount);
		int 	FAS_ClearLatchCount(int iBdID, unsigned int uInputMask);

		int 	FAS_GetOutput(int iBdID, unsigned int* uOutput, unsigned int* uStatus);
		int 	FAS_SetOutput(int iBdID, unsigned int uSet, unsigned int uClear);

		int 	FAS_SetTrigger(int iBdID, unsigned char uOutputNo, TRIGGER_INFO* pTrigger);
		int 	FAS_SetRunStop(int iBdID, unsigned int uRun, unsigned int uStop);
		int 	FAS_GetTriggerCount(int iBdID, unsigned char uOutputNo, unsigned int* uCount);

		int 	FAS_GetIOLevel(int iBdID, unsigned int* uIOLevel);
		int 	FAS_SetIOLevel(int iBdID, unsigned int uIOLevel);
		int 	FAS_LoadIOLevel(int iBdID);
		int 	FAS_SaveIOLevel(int iBdID);

		int 	FAS_GetInputFilter(int iBdID, unsigned short* filter);
		int 	FAS_SetInputFilter(int iBdID, unsigned short filter);

		int 	FAS_GetIODirection(int iBdID, unsigned int* direction);
		int 	FAS_SetIODirection(int iBdID, unsigned int direction);

		//------------------------------------------------------------------
		//			Ezi-IO AD Functions
		//------------------------------------------------------------------
		int 	FAS_SetADConfig(int iBdID, unsigned char channel, enum AD_DATA_TYPE type, int value, int* recv);
		int 	FAS_GetADConfig(int iBdID, unsigned char channel, enum AD_DATA_TYPE type, int* value);

		int 	FAS_LoadADConfig(int iBdID);
		int 	FAS_SaveADConfig(int iBdID);

		int 	FAS_ReadADValue(int iBdID, unsigned char channel, short* advalue);
		int 	FAS_ReadADAllValue(int iBdID, unsigned char offset, AD_BUFFER* adbuffer);

		int 	FAS_GetAllADResult(int iBdID, AD_RESULT* result);
		int 	FAS_GetADResult(int iBdID, unsigned char channel, float* adresult);
		int 	FAS_SetADRange(int iBdID, unsigned char channel, enum AD_RANGE range);

		//------------------------------------------------------------------
		//			Ezi-IO DA Functions
		//------------------------------------------------------------------
		int 	FAS_SetDACConfig(int iBdID, unsigned char channel, DAC_CONFIG type, int data, int* recv);
		int 	FAS_GetDACConfig(int iBdID, unsigned char channel, DAC_CONFIG type, int* data);

		int 	FAS_LoadDACConfig(int iBdID);
		int 	FAS_SaveDACConfig(int iBdID);

		int 	FAS_SetDACValue(int iBdID, unsigned char channel, int bEnable, int value);
		int 	FAS_GetDACValue(int iBdID, unsigned char channel, int* bEnable, int* value);

		//------------------------------------------------------------------
		//			Ezi-IO Counter Functions
		//------------------------------------------------------------------
		int 	FAS_CounterCommand(int iBdID, unsigned char channel, COUNTER_CMD cmd, int data);

		int 	FAS_GetCounterValue(int iBdID, unsigned char channel, COUNTER_VALUE type, int* value);
		int 	FAS_GetCounterStatus(int iBdID, unsigned int* dwStatus);

		int 	FAS_SetCounterTrigger(int iBdID, unsigned char channel, int bStartTrigger, int lStartPos, unsigned int dwPeriod, unsigned int dwPulseTime, unsigned int dwTriggerCnt);

		int 	FAS_SetCounterConfig(int iBdID, unsigned char channel, COUNTER_CONFIG type, int data, int* recv);
		int 	FAS_GetCounterConfig(int iBdID, unsigned char channel, COUNTER_CONFIG type, int* data);

		int 	FAS_LoadCounterConfig(int iBdID);
		int 	FAS_SaveCounterConfig(int iBdID);
	}
}
