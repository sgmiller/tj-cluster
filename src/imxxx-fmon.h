// Generator version : v3.1
// Generation time   : 2025.11.20 11:49:41
// DBC filename      : 20241119 iM-XXX (oil) CAN DB.dbc
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// DBC file version
#define VER_IMXXX_MAJ_FMON (0U)
#define VER_IMXXX_MIN_FMON (0U)

#include "imxxx-config.h"

#ifdef IMXXX_USE_DIAG_MONITORS

#include "canmonitorutil.h"
/*
This file contains the prototypes of all the functions that will be called
from each Unpack_*name* function to detect DBC related errors
It is the user responsibility to defined these functions in the
separated .c file. If it won't be done the linkage error will happen
*/

#ifdef IMXXX_USE_MONO_FMON

void _FMon_MONO_imxxx(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_M160_Temperature_Set_1_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M161_Temperature_Set_2_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M162_Temperature_Set_3_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M163_Analog_Input_Voltages_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M164_Digital_Input_Status_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M165_Motor_Position_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M166_Current_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M167_Voltage_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M168_Flux_ID_IQ_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M169_Internal_Voltages_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M170_Internal_States_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M171_Fault_Codes_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M172_Torque_And_Timer_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M173_Modulation_And_Flux_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M174_Firmware_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M175_Diag_Data_Message_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M176_Fast_Info_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M177_Torque_Capability_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M192_Command_Message_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M193_Read_Write_Param_Command_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_M194_Read_Write_Param_Response_imxxx(x, y) _FMon_MONO_imxxx((x), (y))
#define FMon_BMS_Current_Limit_imxxx(x, y) _FMon_MONO_imxxx((x), (y))

#else

void _FMon_M160_Temperature_Set_1_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M161_Temperature_Set_2_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M162_Temperature_Set_3_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M163_Analog_Input_Voltages_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M164_Digital_Input_Status_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M165_Motor_Position_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M166_Current_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M167_Voltage_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M168_Flux_ID_IQ_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M169_Internal_Voltages_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M170_Internal_States_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M171_Fault_Codes_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M172_Torque_And_Timer_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M173_Modulation_And_Flux_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M174_Firmware_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M175_Diag_Data_Message_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M176_Fast_Info_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M177_Torque_Capability_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M192_Command_Message_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M193_Read_Write_Param_Command_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_M194_Read_Write_Param_Response_imxxx(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_BMS_Current_Limit_imxxx(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_M160_Temperature_Set_1_imxxx(x, y) _FMon_M160_Temperature_Set_1_imxxx((x), (y))
#define FMon_M161_Temperature_Set_2_imxxx(x, y) _FMon_M161_Temperature_Set_2_imxxx((x), (y))
#define FMon_M162_Temperature_Set_3_imxxx(x, y) _FMon_M162_Temperature_Set_3_imxxx((x), (y))
#define FMon_M163_Analog_Input_Voltages_imxxx(x, y) _FMon_M163_Analog_Input_Voltages_imxxx((x), (y))
#define FMon_M164_Digital_Input_Status_imxxx(x, y) _FMon_M164_Digital_Input_Status_imxxx((x), (y))
#define FMon_M165_Motor_Position_Info_imxxx(x, y) _FMon_M165_Motor_Position_Info_imxxx((x), (y))
#define FMon_M166_Current_Info_imxxx(x, y) _FMon_M166_Current_Info_imxxx((x), (y))
#define FMon_M167_Voltage_Info_imxxx(x, y) _FMon_M167_Voltage_Info_imxxx((x), (y))
#define FMon_M168_Flux_ID_IQ_Info_imxxx(x, y) _FMon_M168_Flux_ID_IQ_Info_imxxx((x), (y))
#define FMon_M169_Internal_Voltages_imxxx(x, y) _FMon_M169_Internal_Voltages_imxxx((x), (y))
#define FMon_M170_Internal_States_imxxx(x, y) _FMon_M170_Internal_States_imxxx((x), (y))
#define FMon_M171_Fault_Codes_imxxx(x, y) _FMon_M171_Fault_Codes_imxxx((x), (y))
#define FMon_M172_Torque_And_Timer_Info_imxxx(x, y) _FMon_M172_Torque_And_Timer_Info_imxxx((x), (y))
#define FMon_M173_Modulation_And_Flux_Info_imxxx(x, y) _FMon_M173_Modulation_And_Flux_Info_imxxx((x), (y))
#define FMon_M174_Firmware_Info_imxxx(x, y) _FMon_M174_Firmware_Info_imxxx((x), (y))
#define FMon_M175_Diag_Data_Message_imxxx(x, y) _FMon_M175_Diag_Data_Message_imxxx((x), (y))
#define FMon_M176_Fast_Info_imxxx(x, y) _FMon_M176_Fast_Info_imxxx((x), (y))
#define FMon_M177_Torque_Capability_imxxx(x, y) _FMon_M177_Torque_Capability_imxxx((x), (y))
#define FMon_M192_Command_Message_imxxx(x, y) _FMon_M192_Command_Message_imxxx((x), (y))
#define FMon_M193_Read_Write_Param_Command_imxxx(x, y) _FMon_M193_Read_Write_Param_Command_imxxx((x), (y))
#define FMon_M194_Read_Write_Param_Response_imxxx(x, y) _FMon_M194_Read_Write_Param_Response_imxxx((x), (y))
#define FMon_BMS_Current_Limit_imxxx(x, y) _FMon_BMS_Current_Limit_imxxx((x), (y))

#endif

#endif // IMXXX_USE_DIAG_MONITORS

#ifdef __cplusplus
}
#endif
