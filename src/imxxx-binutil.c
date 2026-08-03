// Generator version : v3.1
// Generation time   : 2025.11.20 11:49:41
// DBC filename      : 20241119 iM-XXX (oil) CAN DB.dbc
#include "imxxx-binutil.h"

// DBC file version
#if (VER_IMXXX_MAJ != (0U)) || (VER_IMXXX_MIN != (0U))
#error The IMXXX binutil source file has inconsistency with core dbc lib!
#endif

#ifdef __DEF_IMXXX__

imxxx_rx_t imxxx_rx;

#endif // __DEF_IMXXX__

uint32_t imxxx_Receive(imxxx_rx_t* _m, const uint8_t* _d, uint32_t _id, uint8_t dlc_)
{
 uint32_t recid = 0;
 if ((_id >= 0xA0U) && (_id < 0xABU)) {
  if ((_id >= 0xA0U) && (_id < 0xA5U)) {
   if ((_id >= 0xA0U) && (_id < 0xA2U)) {
    if (_id == 0xA0U) {
     recid = Unpack_M160_Temperature_Set_1_imXXX(&(_m->M160_Temperature_Set_1), _d, dlc_);
    } else if (_id == 0xA1U) {
     recid = Unpack_M161_Temperature_Set_2_imXXX(&(_m->M161_Temperature_Set_2), _d, dlc_);
    }
   } else {
    if (_id == 0xA2U) {
     recid = Unpack_M162_Temperature_Set_3_imXXX(&(_m->M162_Temperature_Set_3), _d, dlc_);
    } else {
     if (_id == 0xA3U) {
      recid = Unpack_M163_Analog_Input_Voltages_imXXX(&(_m->M163_Analog_Input_Voltages), _d, dlc_);
     } else if (_id == 0xA4U) {
      recid = Unpack_M164_Digital_Input_Status_imXXX(&(_m->M164_Digital_Input_Status), _d, dlc_);
     }
    }
   }
  } else {
   if ((_id >= 0xA5U) && (_id < 0xA8U)) {
    if (_id == 0xA5U) {
     recid = Unpack_M165_Motor_Position_Info_imXXX(&(_m->M165_Motor_Position_Info), _d, dlc_);
    } else {
     if (_id == 0xA6U) {
      recid = Unpack_M166_Current_Info_imXXX(&(_m->M166_Current_Info), _d, dlc_);
     } else if (_id == 0xA7U) {
      recid = Unpack_M167_Voltage_Info_imXXX(&(_m->M167_Voltage_Info), _d, dlc_);
     }
    }
   } else {
    if (_id == 0xA8U) {
     recid = Unpack_M168_Flux_ID_IQ_Info_imXXX(&(_m->M168_Flux_ID_IQ_Info), _d, dlc_);
    } else {
     if (_id == 0xA9U) {
      recid = Unpack_M169_Internal_Voltages_imXXX(&(_m->M169_Internal_Voltages), _d, dlc_);
     } else if (_id == 0xAAU) {
      recid = Unpack_M170_Internal_States_imXXX(&(_m->M170_Internal_States), _d, dlc_);
     }
    }
   }
  }
 } else {
  if ((_id >= 0xABU) && (_id < 0xB0U)) {
   if ((_id >= 0xABU) && (_id < 0xADU)) {
    if (_id == 0xABU) {
     recid = Unpack_M171_Fault_Codes_imXXX(&(_m->M171_Fault_Codes), _d, dlc_);
    } else if (_id == 0xACU) {
     recid = Unpack_M172_Torque_And_Timer_Info_imXXX(&(_m->M172_Torque_And_Timer_Info), _d, dlc_);
    }
   } else {
    if (_id == 0xADU) {
     recid = Unpack_M173_Modulation_And_Flux_Info_imXXX(&(_m->M173_Modulation_And_Flux_Info), _d, dlc_);
    } else {
     if (_id == 0xAEU) {
      recid = Unpack_M174_Firmware_Info_imXXX(&(_m->M174_Firmware_Info), _d, dlc_);
     } else if (_id == 0xAFU) {
      recid = Unpack_M175_Diag_Data_Message_imXXX(&(_m->M175_Diag_Data_Message), _d, dlc_);
     }
    }
   }
  } else {
   if ((_id >= 0xB0U) && (_id < 0xC1U)) {
    if (_id == 0xB0U) {
     recid = Unpack_M176_Fast_Info_imXXX(&(_m->M176_Fast_Info), _d, dlc_);
    } else {
     if (_id == 0xB1U) {
      recid = Unpack_M177_Torque_Capability_imXXX(&(_m->M177_Torque_Capability), _d, dlc_);
     } else if (_id == 0xC0U) {
      recid = Unpack_M192_Command_Message_imXXX(&(_m->M192_Command_Message), _d, dlc_);
     }
    }
   } else {
    if (_id == 0xC1U) {
     recid = Unpack_M193_Read_Write_Param_Command_imXXX(&(_m->M193_Read_Write_Param_Command), _d, dlc_);
    } else {
     if (_id == 0xC2U) {
      recid = Unpack_M194_Read_Write_Param_Response_imXXX(&(_m->M194_Read_Write_Param_Response), _d, dlc_);
     } else if (_id == 0x202U) {
      recid = Unpack_BMS_Current_Limit_imXXX(&(_m->BMS_Current_Limit), _d, dlc_);
     }
    }
   }
  }
 }

 return recid;
}

