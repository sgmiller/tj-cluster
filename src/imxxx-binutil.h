// Generator version : v3.1
// Generation time   : 2025.11.20 11:49:41
// DBC filename      : 20241119 iM-XXX (oil) CAN DB.dbc
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dbccodeconf.h"

#include "imxxx.h"

typedef struct
{
  M160_Temperature_Set_1_t M160_Temperature_Set_1;
  M161_Temperature_Set_2_t M161_Temperature_Set_2;
  M162_Temperature_Set_3_t M162_Temperature_Set_3;
  M163_Analog_Input_Voltages_t M163_Analog_Input_Voltages;
  M164_Digital_Input_Status_t M164_Digital_Input_Status;
  M165_Motor_Position_Info_t M165_Motor_Position_Info;
  M166_Current_Info_t M166_Current_Info;
  M167_Voltage_Info_t M167_Voltage_Info;
  M168_Flux_ID_IQ_Info_t M168_Flux_ID_IQ_Info;
  M169_Internal_Voltages_t M169_Internal_Voltages;
  M170_Internal_States_t M170_Internal_States;
  M171_Fault_Codes_t M171_Fault_Codes;
  M172_Torque_And_Timer_Info_t M172_Torque_And_Timer_Info;
  M173_Modulation_And_Flux_Info_t M173_Modulation_And_Flux_Info;
  M174_Firmware_Info_t M174_Firmware_Info;
  M175_Diag_Data_Message_t M175_Diag_Data_Message;
  M176_Fast_Info_t M176_Fast_Info;
  M177_Torque_Capability_t M177_Torque_Capability;
  M192_Command_Message_t M192_Command_Message;
  M193_Read_Write_Param_Command_t M193_Read_Write_Param_Command;
  M194_Read_Write_Param_Response_t M194_Read_Write_Param_Response;
  BMS_Current_Limit_t BMS_Current_Limit;
} imxxx_rx_t;

// There is no any TX mapped massage.

uint32_t imxxx_Receive(imxxx_rx_t* m, const uint8_t* d, uint32_t msgid, uint8_t dlc);

#ifdef __DEF_IMXXX__

extern imxxx_rx_t imxxx_rx;

#endif // __DEF_IMXXX__

#ifdef __cplusplus
}
#endif
