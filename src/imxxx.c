// Generator version : v3.1
// Generation time   : 2025.11.20 11:49:41
// DBC filename      : 20241119 iM-XXX (oil) CAN DB.dbc
#include "imxxx.h"


// DBC file version
#if (VER_IMXXX_MAJ != (0U)) || (VER_IMXXX_MIN != (0U))
#error The IMXXX dbc source files have different versions
#endif

#ifdef IMXXX_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "imxxx-fmon.h"

#endif // IMXXX_USE_DIAG_MONITORS

// This macro guard for the case when you need to enable
// using diag monitors but there is no necessity in proper
// SysTick provider. For providing one you need define macro
// before this line - in dbccodeconf.h

#ifndef GetSystemTick
#define GetSystemTick() (0u)
#endif

// This macro guard is for the case when you want to build
// app with enabled optoin auto CSM, but don't yet have
// proper getframehash implementation

#ifndef GetFrameHash
#define GetFrameHash(a,b,c,d,e) (0u)
#endif

// This function performs extension of sign for the signals
// whose bit width value is not aligned to one of power of 2 or less than 8.
// The types 'bitext_t' and 'ubitext_t' define the biggest bit width which
// can be correctly handled. You need to select type which can contain
// n+1 bits where n is the largest signed signal width. For example if
// the most wide signed signal has a width of 31 bits you need to set
// bitext_t as int32_t and ubitext_t as uint32_t
// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h
static bitext_t __ext_sig__(ubitext_t val, uint8_t bits)
{
  ubitext_t const m = (ubitext_t) (1u << (bits - 1u));
  return ((val ^ m) - m);
}

uint32_t Unpack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Module_A_Temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Module_A_Temp_phys = (sigfloat_t)(IMXXX_INV_Module_A_Temp_ro_fromS(_m->INV_Module_A_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Module_B_Temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Module_B_Temp_phys = (sigfloat_t)(IMXXX_INV_Module_B_Temp_ro_fromS(_m->INV_Module_B_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Module_C_Temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Module_C_Temp_phys = (sigfloat_t)(IMXXX_INV_Module_C_Temp_ro_fromS(_m->INV_Module_C_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Gate_Driver_Board_Temp_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Gate_Driver_Board_Temp_phys = (sigfloat_t)(IMXXX_INV_Gate_Driver_Board_Temp_ro_fromS(_m->INV_Gate_Driver_Board_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M160_Temperature_Set_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M160_Temperature_Set_1_imxxx(&_m->mon1, M160_Temperature_Set_1_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M160_Temperature_Set_1_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M160_Temperature_Set_1_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Module_A_Temp_ro = (int16_t) IMXXX_INV_Module_A_Temp_ro_toS(_m->INV_Module_A_Temp_phys);
  _m->INV_Module_B_Temp_ro = (int16_t) IMXXX_INV_Module_B_Temp_ro_toS(_m->INV_Module_B_Temp_phys);
  _m->INV_Module_C_Temp_ro = (int16_t) IMXXX_INV_Module_C_Temp_ro_toS(_m->INV_Module_C_Temp_phys);
  _m->INV_Gate_Driver_Board_Temp_ro = (int16_t) IMXXX_INV_Gate_Driver_Board_Temp_ro_toS(_m->INV_Gate_Driver_Board_Temp_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Module_A_Temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Module_A_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Module_B_Temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Module_B_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Module_C_Temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Module_C_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Gate_Driver_Board_Temp_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Gate_Driver_Board_Temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M160_Temperature_Set_1_CANID;
  cframe->DLC = (uint8_t) M160_Temperature_Set_1_DLC;
  cframe->IDE = (uint8_t) M160_Temperature_Set_1_IDE;
  return M160_Temperature_Set_1_CANID;
}

#else

uint32_t Pack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M160_Temperature_Set_1_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Module_A_Temp_ro = (int16_t) IMXXX_INV_Module_A_Temp_ro_toS(_m->INV_Module_A_Temp_phys);
  _m->INV_Module_B_Temp_ro = (int16_t) IMXXX_INV_Module_B_Temp_ro_toS(_m->INV_Module_B_Temp_phys);
  _m->INV_Module_C_Temp_ro = (int16_t) IMXXX_INV_Module_C_Temp_ro_toS(_m->INV_Module_C_Temp_phys);
  _m->INV_Gate_Driver_Board_Temp_ro = (int16_t) IMXXX_INV_Gate_Driver_Board_Temp_ro_toS(_m->INV_Gate_Driver_Board_Temp_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Module_A_Temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Module_A_Temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Module_B_Temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Module_B_Temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Module_C_Temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Module_C_Temp_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Gate_Driver_Board_Temp_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Gate_Driver_Board_Temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M160_Temperature_Set_1_DLC;
  *_ide = (uint8_t) M160_Temperature_Set_1_IDE;
  return M160_Temperature_Set_1_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Control_Board_Temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Control_Board_Temp_phys = (sigfloat_t)(IMXXX_INV_Control_Board_Temp_ro_fromS(_m->INV_Control_Board_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_RTD1_Temperature_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_RTD1_Temperature_phys = (sigfloat_t)(IMXXX_INV_RTD1_Temperature_ro_fromS(_m->INV_RTD1_Temperature_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_RTD2_Temperature_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_RTD2_Temperature_phys = (sigfloat_t)(IMXXX_INV_RTD2_Temperature_ro_fromS(_m->INV_RTD2_Temperature_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Stall_Burst_Model_Temp_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Stall_Burst_Model_Temp_phys = (sigfloat_t)(IMXXX_INV_Stall_Burst_Model_Temp_ro_fromS(_m->INV_Stall_Burst_Model_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M161_Temperature_Set_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M161_Temperature_Set_2_imxxx(&_m->mon1, M161_Temperature_Set_2_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M161_Temperature_Set_2_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M161_Temperature_Set_2_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Control_Board_Temp_ro = (int16_t) IMXXX_INV_Control_Board_Temp_ro_toS(_m->INV_Control_Board_Temp_phys);
  _m->INV_RTD1_Temperature_ro = (int16_t) IMXXX_INV_RTD1_Temperature_ro_toS(_m->INV_RTD1_Temperature_phys);
  _m->INV_RTD2_Temperature_ro = (int16_t) IMXXX_INV_RTD2_Temperature_ro_toS(_m->INV_RTD2_Temperature_phys);
  _m->INV_Stall_Burst_Model_Temp_ro = (int16_t) IMXXX_INV_Stall_Burst_Model_Temp_ro_toS(_m->INV_Stall_Burst_Model_Temp_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Control_Board_Temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Control_Board_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_RTD1_Temperature_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_RTD1_Temperature_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_RTD2_Temperature_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_RTD2_Temperature_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Stall_Burst_Model_Temp_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Stall_Burst_Model_Temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M161_Temperature_Set_2_CANID;
  cframe->DLC = (uint8_t) M161_Temperature_Set_2_DLC;
  cframe->IDE = (uint8_t) M161_Temperature_Set_2_IDE;
  return M161_Temperature_Set_2_CANID;
}

#else

uint32_t Pack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M161_Temperature_Set_2_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Control_Board_Temp_ro = (int16_t) IMXXX_INV_Control_Board_Temp_ro_toS(_m->INV_Control_Board_Temp_phys);
  _m->INV_RTD1_Temperature_ro = (int16_t) IMXXX_INV_RTD1_Temperature_ro_toS(_m->INV_RTD1_Temperature_phys);
  _m->INV_RTD2_Temperature_ro = (int16_t) IMXXX_INV_RTD2_Temperature_ro_toS(_m->INV_RTD2_Temperature_phys);
  _m->INV_Stall_Burst_Model_Temp_ro = (int16_t) IMXXX_INV_Stall_Burst_Model_Temp_ro_toS(_m->INV_Stall_Burst_Model_Temp_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Control_Board_Temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Control_Board_Temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_RTD1_Temperature_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_RTD1_Temperature_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_RTD2_Temperature_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_RTD2_Temperature_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Stall_Burst_Model_Temp_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Stall_Burst_Model_Temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M161_Temperature_Set_2_DLC;
  *_ide = (uint8_t) M161_Temperature_Set_2_IDE;
  return M161_Temperature_Set_2_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Coolant_Temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Coolant_Temp_phys = (sigfloat_t)(IMXXX_INV_Coolant_Temp_ro_fromS(_m->INV_Coolant_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Hot_Spot_Temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Hot_Spot_Temp_phys = (sigfloat_t)(IMXXX_INV_Hot_Spot_Temp_ro_fromS(_m->INV_Hot_Spot_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Motor_Temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Motor_Temp_phys = (sigfloat_t)(IMXXX_INV_Motor_Temp_ro_fromS(_m->INV_Motor_Temp_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Torque_Shudder_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Shudder_phys = (sigfloat_t)(IMXXX_INV_Torque_Shudder_ro_fromS(_m->INV_Torque_Shudder_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M162_Temperature_Set_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M162_Temperature_Set_3_imxxx(&_m->mon1, M162_Temperature_Set_3_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M162_Temperature_Set_3_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M162_Temperature_Set_3_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Coolant_Temp_ro = (int16_t) IMXXX_INV_Coolant_Temp_ro_toS(_m->INV_Coolant_Temp_phys);
  _m->INV_Hot_Spot_Temp_ro = (int16_t) IMXXX_INV_Hot_Spot_Temp_ro_toS(_m->INV_Hot_Spot_Temp_phys);
  _m->INV_Motor_Temp_ro = (int16_t) IMXXX_INV_Motor_Temp_ro_toS(_m->INV_Motor_Temp_phys);
  _m->INV_Torque_Shudder_ro = (int16_t) IMXXX_INV_Torque_Shudder_ro_toS(_m->INV_Torque_Shudder_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Coolant_Temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Coolant_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Hot_Spot_Temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Hot_Spot_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Motor_Temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Motor_Temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Torque_Shudder_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Torque_Shudder_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M162_Temperature_Set_3_CANID;
  cframe->DLC = (uint8_t) M162_Temperature_Set_3_DLC;
  cframe->IDE = (uint8_t) M162_Temperature_Set_3_IDE;
  return M162_Temperature_Set_3_CANID;
}

#else

uint32_t Pack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M162_Temperature_Set_3_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Coolant_Temp_ro = (int16_t) IMXXX_INV_Coolant_Temp_ro_toS(_m->INV_Coolant_Temp_phys);
  _m->INV_Hot_Spot_Temp_ro = (int16_t) IMXXX_INV_Hot_Spot_Temp_ro_toS(_m->INV_Hot_Spot_Temp_phys);
  _m->INV_Motor_Temp_ro = (int16_t) IMXXX_INV_Motor_Temp_ro_toS(_m->INV_Motor_Temp_phys);
  _m->INV_Torque_Shudder_ro = (int16_t) IMXXX_INV_Torque_Shudder_ro_toS(_m->INV_Torque_Shudder_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Coolant_Temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Coolant_Temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Hot_Spot_Temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Hot_Spot_Temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Motor_Temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Motor_Temp_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Torque_Shudder_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Torque_Shudder_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M162_Temperature_Set_3_DLC;
  *_ide = (uint8_t) M162_Temperature_Set_3_IDE;
  return M162_Temperature_Set_3_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Oil_Temperature_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Oil_Temperature_phys = (sigfloat_t)(IMXXX_INV_Oil_Temperature_ro_fromS(_m->INV_Oil_Temperature_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Oil_Pressure_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Oil_Pressure_phys = (sigfloat_t)(IMXXX_INV_Oil_Pressure_ro_fromS(_m->INV_Oil_Pressure_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Analog_Input_4_ro = (uint16_t) ( ((_d[5] & (0x03U)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Analog_Input_4_phys = (sigfloat_t)(IMXXX_INV_Analog_Input_4_ro_fromS(_m->INV_Analog_Input_4_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Analog_Input_5_ro = (uint16_t) ( ((_d[6] & (0x0FU)) << 6U) | ((_d[5] >> 2U) & (0x3FU)) );
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Analog_Input_5_phys = (sigfloat_t)(IMXXX_INV_Analog_Input_5_ro_fromS(_m->INV_Analog_Input_5_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Analog_Input_6_ro = (uint16_t) ( ((_d[7] & (0x3FU)) << 4U) | ((_d[6] >> 4U) & (0x0FU)) );
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Analog_Input_6_phys = (sigfloat_t)(IMXXX_INV_Analog_Input_6_ro_fromS(_m->INV_Analog_Input_6_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M163_Analog_Input_Voltages_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M163_Analog_Input_Voltages_imxxx(&_m->mon1, M163_Analog_Input_Voltages_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M163_Analog_Input_Voltages_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M163_Analog_Input_Voltages_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Oil_Temperature_ro = (int16_t) IMXXX_INV_Oil_Temperature_ro_toS(_m->INV_Oil_Temperature_phys);
  _m->INV_Oil_Pressure_ro = (int16_t) IMXXX_INV_Oil_Pressure_ro_toS(_m->INV_Oil_Pressure_phys);
  _m->INV_Analog_Input_4_ro = (uint16_t) IMXXX_INV_Analog_Input_4_ro_toS(_m->INV_Analog_Input_4_phys);
  _m->INV_Analog_Input_5_ro = (uint16_t) IMXXX_INV_Analog_Input_5_ro_toS(_m->INV_Analog_Input_5_phys);
  _m->INV_Analog_Input_6_ro = (uint16_t) IMXXX_INV_Analog_Input_6_ro_toS(_m->INV_Analog_Input_6_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Oil_Temperature_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Oil_Temperature_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Oil_Pressure_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Oil_Pressure_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Analog_Input_4_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Analog_Input_4_ro >> 8U) & (0x03U)) | ((_m->INV_Analog_Input_5_ro & (0x3FU)) << 2U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->INV_Analog_Input_5_ro >> 6U) & (0x0FU)) | ((_m->INV_Analog_Input_6_ro & (0x0FU)) << 4U) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Analog_Input_6_ro >> 4U) & (0x3FU)) );

  cframe->MsgId = (uint32_t) M163_Analog_Input_Voltages_CANID;
  cframe->DLC = (uint8_t) M163_Analog_Input_Voltages_DLC;
  cframe->IDE = (uint8_t) M163_Analog_Input_Voltages_IDE;
  return M163_Analog_Input_Voltages_CANID;
}

#else

uint32_t Pack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M163_Analog_Input_Voltages_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Oil_Temperature_ro = (int16_t) IMXXX_INV_Oil_Temperature_ro_toS(_m->INV_Oil_Temperature_phys);
  _m->INV_Oil_Pressure_ro = (int16_t) IMXXX_INV_Oil_Pressure_ro_toS(_m->INV_Oil_Pressure_phys);
  _m->INV_Analog_Input_4_ro = (uint16_t) IMXXX_INV_Analog_Input_4_ro_toS(_m->INV_Analog_Input_4_phys);
  _m->INV_Analog_Input_5_ro = (uint16_t) IMXXX_INV_Analog_Input_5_ro_toS(_m->INV_Analog_Input_5_phys);
  _m->INV_Analog_Input_6_ro = (uint16_t) IMXXX_INV_Analog_Input_6_ro_toS(_m->INV_Analog_Input_6_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Oil_Temperature_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Oil_Temperature_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Oil_Pressure_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Oil_Pressure_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Analog_Input_4_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Analog_Input_4_ro >> 8U) & (0x03U)) | ((_m->INV_Analog_Input_5_ro & (0x3FU)) << 2U) );
  _d[6] |= (uint8_t) ( ((_m->INV_Analog_Input_5_ro >> 6U) & (0x0FU)) | ((_m->INV_Analog_Input_6_ro & (0x0FU)) << 4U) );
  _d[7] |= (uint8_t) ( ((_m->INV_Analog_Input_6_ro >> 4U) & (0x3FU)) );

  *_len = (uint8_t) M163_Analog_Input_Voltages_DLC;
  *_ide = (uint8_t) M163_Analog_Input_Voltages_IDE;
  return M163_Analog_Input_Voltages_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Digital_Input_1 = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->INV_Digital_Input_2 = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->INV_Digital_Input_3 = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->INV_Digital_Input_4 = (uint8_t) ( (_d[3] & (0x01U)) );
  _m->INV_Digital_Input_5 = (uint8_t) ( (_d[4] & (0x01U)) );
  _m->INV_Digital_Input_6 = (uint8_t) ( (_d[5] & (0x01U)) );
  _m->INV_Digital_Input_7 = (uint8_t) ( (_d[6] & (0x01U)) );
  _m->INV_Digital_Input_8 = (uint8_t) ( (_d[7] & (0x01U)) );

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M164_Digital_Input_Status_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M164_Digital_Input_Status_imxxx(&_m->mon1, M164_Digital_Input_Status_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M164_Digital_Input_Status_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M164_Digital_Input_Status_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Digital_Input_1 & (0x01U)) );
  cframe->Data[1] |= (uint8_t) ( (_m->INV_Digital_Input_2 & (0x01U)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Digital_Input_3 & (0x01U)) );
  cframe->Data[3] |= (uint8_t) ( (_m->INV_Digital_Input_4 & (0x01U)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Digital_Input_5 & (0x01U)) );
  cframe->Data[5] |= (uint8_t) ( (_m->INV_Digital_Input_6 & (0x01U)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Digital_Input_7 & (0x01U)) );
  cframe->Data[7] |= (uint8_t) ( (_m->INV_Digital_Input_8 & (0x01U)) );

  cframe->MsgId = (uint32_t) M164_Digital_Input_Status_CANID;
  cframe->DLC = (uint8_t) M164_Digital_Input_Status_DLC;
  cframe->IDE = (uint8_t) M164_Digital_Input_Status_IDE;
  return M164_Digital_Input_Status_CANID;
}

#else

uint32_t Pack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M164_Digital_Input_Status_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->INV_Digital_Input_1 & (0x01U)) );
  _d[1] |= (uint8_t) ( (_m->INV_Digital_Input_2 & (0x01U)) );
  _d[2] |= (uint8_t) ( (_m->INV_Digital_Input_3 & (0x01U)) );
  _d[3] |= (uint8_t) ( (_m->INV_Digital_Input_4 & (0x01U)) );
  _d[4] |= (uint8_t) ( (_m->INV_Digital_Input_5 & (0x01U)) );
  _d[5] |= (uint8_t) ( (_m->INV_Digital_Input_6 & (0x01U)) );
  _d[6] |= (uint8_t) ( (_m->INV_Digital_Input_7 & (0x01U)) );
  _d[7] |= (uint8_t) ( (_m->INV_Digital_Input_8 & (0x01U)) );

  *_len = (uint8_t) M164_Digital_Input_Status_DLC;
  *_ide = (uint8_t) M164_Digital_Input_Status_IDE;
  return M164_Digital_Input_Status_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Motor_Angle_Electrical_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Motor_Angle_Electrical_phys = (sigfloat_t)(IMXXX_INV_Motor_Angle_Electrical_ro_fromS(_m->INV_Motor_Angle_Electrical_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Motor_Speed = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->INV_Electrical_Output_Frequency_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Electrical_Output_Frequency_phys = (sigfloat_t)(IMXXX_INV_Electrical_Output_Frequency_ro_fromS(_m->INV_Electrical_Output_Frequency_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Delta_Resolver_Filtered_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Delta_Resolver_Filtered_phys = (sigfloat_t)(IMXXX_INV_Delta_Resolver_Filtered_ro_fromS(_m->INV_Delta_Resolver_Filtered_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M165_Motor_Position_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M165_Motor_Position_Info_imxxx(&_m->mon1, M165_Motor_Position_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M165_Motor_Position_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M165_Motor_Position_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Motor_Angle_Electrical_ro = (uint16_t) IMXXX_INV_Motor_Angle_Electrical_ro_toS(_m->INV_Motor_Angle_Electrical_phys);
  _m->INV_Electrical_Output_Frequency_ro = (int16_t) IMXXX_INV_Electrical_Output_Frequency_ro_toS(_m->INV_Electrical_Output_Frequency_phys);
  _m->INV_Delta_Resolver_Filtered_ro = (int16_t) IMXXX_INV_Delta_Resolver_Filtered_ro_toS(_m->INV_Delta_Resolver_Filtered_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Motor_Angle_Electrical_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Motor_Angle_Electrical_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Motor_Speed & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Motor_Speed >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Electrical_Output_Frequency_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Electrical_Output_Frequency_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Delta_Resolver_Filtered_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Delta_Resolver_Filtered_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M165_Motor_Position_Info_CANID;
  cframe->DLC = (uint8_t) M165_Motor_Position_Info_DLC;
  cframe->IDE = (uint8_t) M165_Motor_Position_Info_IDE;
  return M165_Motor_Position_Info_CANID;
}

#else

uint32_t Pack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M165_Motor_Position_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Motor_Angle_Electrical_ro = (uint16_t) IMXXX_INV_Motor_Angle_Electrical_ro_toS(_m->INV_Motor_Angle_Electrical_phys);
  _m->INV_Electrical_Output_Frequency_ro = (int16_t) IMXXX_INV_Electrical_Output_Frequency_ro_toS(_m->INV_Electrical_Output_Frequency_phys);
  _m->INV_Delta_Resolver_Filtered_ro = (int16_t) IMXXX_INV_Delta_Resolver_Filtered_ro_toS(_m->INV_Delta_Resolver_Filtered_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Motor_Angle_Electrical_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Motor_Angle_Electrical_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Motor_Speed & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Motor_Speed >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Electrical_Output_Frequency_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Electrical_Output_Frequency_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Delta_Resolver_Filtered_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Delta_Resolver_Filtered_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M165_Motor_Position_Info_DLC;
  *_ide = (uint8_t) M165_Motor_Position_Info_IDE;
  return M165_Motor_Position_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Phase_A_Current_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Phase_A_Current_phys = (sigfloat_t)(IMXXX_INV_Phase_A_Current_ro_fromS(_m->INV_Phase_A_Current_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Phase_B_Current_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Phase_B_Current_phys = (sigfloat_t)(IMXXX_INV_Phase_B_Current_ro_fromS(_m->INV_Phase_B_Current_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Phase_C_Current_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Phase_C_Current_phys = (sigfloat_t)(IMXXX_INV_Phase_C_Current_ro_fromS(_m->INV_Phase_C_Current_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_DC_Bus_Current_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_DC_Bus_Current_phys = (sigfloat_t)(IMXXX_INV_DC_Bus_Current_ro_fromS(_m->INV_DC_Bus_Current_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M166_Current_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M166_Current_Info_imxxx(&_m->mon1, M166_Current_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M166_Current_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M166_Current_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Phase_A_Current_ro = (int16_t) IMXXX_INV_Phase_A_Current_ro_toS(_m->INV_Phase_A_Current_phys);
  _m->INV_Phase_B_Current_ro = (int16_t) IMXXX_INV_Phase_B_Current_ro_toS(_m->INV_Phase_B_Current_phys);
  _m->INV_Phase_C_Current_ro = (int16_t) IMXXX_INV_Phase_C_Current_ro_toS(_m->INV_Phase_C_Current_phys);
  _m->INV_DC_Bus_Current_ro = (int16_t) IMXXX_INV_DC_Bus_Current_ro_toS(_m->INV_DC_Bus_Current_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Phase_A_Current_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Phase_A_Current_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Phase_B_Current_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Phase_B_Current_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Phase_C_Current_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Phase_C_Current_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_DC_Bus_Current_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_DC_Bus_Current_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M166_Current_Info_CANID;
  cframe->DLC = (uint8_t) M166_Current_Info_DLC;
  cframe->IDE = (uint8_t) M166_Current_Info_IDE;
  return M166_Current_Info_CANID;
}

#else

uint32_t Pack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M166_Current_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Phase_A_Current_ro = (int16_t) IMXXX_INV_Phase_A_Current_ro_toS(_m->INV_Phase_A_Current_phys);
  _m->INV_Phase_B_Current_ro = (int16_t) IMXXX_INV_Phase_B_Current_ro_toS(_m->INV_Phase_B_Current_phys);
  _m->INV_Phase_C_Current_ro = (int16_t) IMXXX_INV_Phase_C_Current_ro_toS(_m->INV_Phase_C_Current_phys);
  _m->INV_DC_Bus_Current_ro = (int16_t) IMXXX_INV_DC_Bus_Current_ro_toS(_m->INV_DC_Bus_Current_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Phase_A_Current_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Phase_A_Current_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Phase_B_Current_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Phase_B_Current_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Phase_C_Current_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Phase_C_Current_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_DC_Bus_Current_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_DC_Bus_Current_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M166_Current_Info_DLC;
  *_ide = (uint8_t) M166_Current_Info_IDE;
  return M166_Current_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_DC_Bus_Voltage_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_DC_Bus_Voltage_phys = (sigfloat_t)(IMXXX_INV_DC_Bus_Voltage_ro_fromS(_m->INV_DC_Bus_Voltage_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Output_Voltage_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Output_Voltage_phys = (sigfloat_t)(IMXXX_INV_Output_Voltage_ro_fromS(_m->INV_Output_Voltage_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_VAB_Vd_Voltage_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_VAB_Vd_Voltage_phys = (sigfloat_t)(IMXXX_INV_VAB_Vd_Voltage_ro_fromS(_m->INV_VAB_Vd_Voltage_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_VBC_Vq_Voltage_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_VBC_Vq_Voltage_phys = (sigfloat_t)(IMXXX_INV_VBC_Vq_Voltage_ro_fromS(_m->INV_VBC_Vq_Voltage_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M167_Voltage_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M167_Voltage_Info_imxxx(&_m->mon1, M167_Voltage_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M167_Voltage_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M167_Voltage_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_DC_Bus_Voltage_ro = (int16_t) IMXXX_INV_DC_Bus_Voltage_ro_toS(_m->INV_DC_Bus_Voltage_phys);
  _m->INV_Output_Voltage_ro = (int16_t) IMXXX_INV_Output_Voltage_ro_toS(_m->INV_Output_Voltage_phys);
  _m->INV_VAB_Vd_Voltage_ro = (int16_t) IMXXX_INV_VAB_Vd_Voltage_ro_toS(_m->INV_VAB_Vd_Voltage_phys);
  _m->INV_VBC_Vq_Voltage_ro = (int16_t) IMXXX_INV_VBC_Vq_Voltage_ro_toS(_m->INV_VBC_Vq_Voltage_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_DC_Bus_Voltage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_DC_Bus_Voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Output_Voltage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Output_Voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_VAB_Vd_Voltage_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_VAB_Vd_Voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_VBC_Vq_Voltage_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_VBC_Vq_Voltage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M167_Voltage_Info_CANID;
  cframe->DLC = (uint8_t) M167_Voltage_Info_DLC;
  cframe->IDE = (uint8_t) M167_Voltage_Info_IDE;
  return M167_Voltage_Info_CANID;
}

#else

uint32_t Pack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M167_Voltage_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_DC_Bus_Voltage_ro = (int16_t) IMXXX_INV_DC_Bus_Voltage_ro_toS(_m->INV_DC_Bus_Voltage_phys);
  _m->INV_Output_Voltage_ro = (int16_t) IMXXX_INV_Output_Voltage_ro_toS(_m->INV_Output_Voltage_phys);
  _m->INV_VAB_Vd_Voltage_ro = (int16_t) IMXXX_INV_VAB_Vd_Voltage_ro_toS(_m->INV_VAB_Vd_Voltage_phys);
  _m->INV_VBC_Vq_Voltage_ro = (int16_t) IMXXX_INV_VBC_Vq_Voltage_ro_toS(_m->INV_VBC_Vq_Voltage_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_DC_Bus_Voltage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_DC_Bus_Voltage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Output_Voltage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Output_Voltage_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_VAB_Vd_Voltage_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_VAB_Vd_Voltage_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_VBC_Vq_Voltage_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_VBC_Vq_Voltage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M167_Voltage_Info_DLC;
  *_ide = (uint8_t) M167_Voltage_Info_IDE;
  return M167_Voltage_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Vd_ff_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Vd_ff_phys = (sigfloat_t)(IMXXX_INV_Vd_ff_ro_fromS(_m->INV_Vd_ff_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Vq_ff_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Vq_ff_phys = (sigfloat_t)(IMXXX_INV_Vq_ff_ro_fromS(_m->INV_Vq_ff_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Id_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Id_phys = (sigfloat_t)(IMXXX_INV_Id_ro_fromS(_m->INV_Id_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Iq_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Iq_phys = (sigfloat_t)(IMXXX_INV_Iq_ro_fromS(_m->INV_Iq_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M168_Flux_ID_IQ_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M168_Flux_ID_IQ_Info_imxxx(&_m->mon1, M168_Flux_ID_IQ_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M168_Flux_ID_IQ_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M168_Flux_ID_IQ_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Vd_ff_ro = (int16_t) IMXXX_INV_Vd_ff_ro_toS(_m->INV_Vd_ff_phys);
  _m->INV_Vq_ff_ro = (int16_t) IMXXX_INV_Vq_ff_ro_toS(_m->INV_Vq_ff_phys);
  _m->INV_Id_ro = (int16_t) IMXXX_INV_Id_ro_toS(_m->INV_Id_phys);
  _m->INV_Iq_ro = (int16_t) IMXXX_INV_Iq_ro_toS(_m->INV_Iq_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Vd_ff_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Vd_ff_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Vq_ff_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Vq_ff_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Id_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Id_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Iq_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Iq_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M168_Flux_ID_IQ_Info_CANID;
  cframe->DLC = (uint8_t) M168_Flux_ID_IQ_Info_DLC;
  cframe->IDE = (uint8_t) M168_Flux_ID_IQ_Info_IDE;
  return M168_Flux_ID_IQ_Info_CANID;
}

#else

uint32_t Pack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M168_Flux_ID_IQ_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Vd_ff_ro = (int16_t) IMXXX_INV_Vd_ff_ro_toS(_m->INV_Vd_ff_phys);
  _m->INV_Vq_ff_ro = (int16_t) IMXXX_INV_Vq_ff_ro_toS(_m->INV_Vq_ff_phys);
  _m->INV_Id_ro = (int16_t) IMXXX_INV_Id_ro_toS(_m->INV_Id_phys);
  _m->INV_Iq_ro = (int16_t) IMXXX_INV_Iq_ro_toS(_m->INV_Iq_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Vd_ff_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Vd_ff_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Vq_ff_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Vq_ff_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Id_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Id_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Iq_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Iq_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M168_Flux_ID_IQ_Info_DLC;
  *_ide = (uint8_t) M168_Flux_ID_IQ_Info_IDE;
  return M168_Flux_ID_IQ_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Ref_Voltage_1_5_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_1_5_phys = (sigfloat_t)(IMXXX_INV_Ref_Voltage_1_5_ro_fromS(_m->INV_Ref_Voltage_1_5_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Ref_Voltage_2_5_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_2_5_phys = (sigfloat_t)(IMXXX_INV_Ref_Voltage_2_5_ro_fromS(_m->INV_Ref_Voltage_2_5_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Ref_Voltage_5_0_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_5_0_phys = (sigfloat_t)(IMXXX_INV_Ref_Voltage_5_0_ro_fromS(_m->INV_Ref_Voltage_5_0_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Ref_Voltage_12_0_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_12_0_phys = (sigfloat_t)(IMXXX_INV_Ref_Voltage_12_0_ro_fromS(_m->INV_Ref_Voltage_12_0_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M169_Internal_Voltages_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M169_Internal_Voltages_imxxx(&_m->mon1, M169_Internal_Voltages_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M169_Internal_Voltages_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M169_Internal_Voltages_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_1_5_ro = (int16_t) IMXXX_INV_Ref_Voltage_1_5_ro_toS(_m->INV_Ref_Voltage_1_5_phys);
  _m->INV_Ref_Voltage_2_5_ro = (int16_t) IMXXX_INV_Ref_Voltage_2_5_ro_toS(_m->INV_Ref_Voltage_2_5_phys);
  _m->INV_Ref_Voltage_5_0_ro = (int16_t) IMXXX_INV_Ref_Voltage_5_0_ro_toS(_m->INV_Ref_Voltage_5_0_phys);
  _m->INV_Ref_Voltage_12_0_ro = (int16_t) IMXXX_INV_Ref_Voltage_12_0_ro_toS(_m->INV_Ref_Voltage_12_0_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Ref_Voltage_1_5_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Ref_Voltage_1_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Ref_Voltage_2_5_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Ref_Voltage_2_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Ref_Voltage_5_0_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Ref_Voltage_5_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Ref_Voltage_12_0_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Ref_Voltage_12_0_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M169_Internal_Voltages_CANID;
  cframe->DLC = (uint8_t) M169_Internal_Voltages_DLC;
  cframe->IDE = (uint8_t) M169_Internal_Voltages_IDE;
  return M169_Internal_Voltages_CANID;
}

#else

uint32_t Pack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M169_Internal_Voltages_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Ref_Voltage_1_5_ro = (int16_t) IMXXX_INV_Ref_Voltage_1_5_ro_toS(_m->INV_Ref_Voltage_1_5_phys);
  _m->INV_Ref_Voltage_2_5_ro = (int16_t) IMXXX_INV_Ref_Voltage_2_5_ro_toS(_m->INV_Ref_Voltage_2_5_phys);
  _m->INV_Ref_Voltage_5_0_ro = (int16_t) IMXXX_INV_Ref_Voltage_5_0_ro_toS(_m->INV_Ref_Voltage_5_0_phys);
  _m->INV_Ref_Voltage_12_0_ro = (int16_t) IMXXX_INV_Ref_Voltage_12_0_ro_toS(_m->INV_Ref_Voltage_12_0_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Ref_Voltage_1_5_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Ref_Voltage_1_5_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Ref_Voltage_2_5_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Ref_Voltage_2_5_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Ref_Voltage_5_0_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Ref_Voltage_5_0_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Ref_Voltage_12_0_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Ref_Voltage_12_0_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M169_Internal_Voltages_DLC;
  *_ide = (uint8_t) M169_Internal_Voltages_IDE;
  return M169_Internal_Voltages_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_VSM_State = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->INV_PWM_Frequency = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->INV_Inverter_State = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->INV_Relay_1_Status = (uint8_t) ( (_d[3] & (0x01U)) );
  _m->INV_Relay_2_Status = (uint8_t) ( ((_d[3] >> 1U) & (0x01U)) );
  _m->INV_Relay_3_Status = (uint8_t) ( ((_d[3] >> 2U) & (0x01U)) );
  _m->INV_Relay_4_Status = (uint8_t) ( ((_d[3] >> 3U) & (0x01U)) );
  _m->INV_Relay_5_Status = (uint8_t) ( ((_d[3] >> 4U) & (0x01U)) );
  _m->INV_Relay_6_Status = (uint8_t) ( ((_d[3] >> 5U) & (0x01U)) );
  _m->INV_Inverter_Run_Mode = (uint8_t) ( (_d[4] & (0x01U)) );
  _m->INV_Self_Sensing_Assist_Enable = (uint8_t) ( ((_d[4] >> 1U) & (0x01U)) );
  _m->INV_Inverter_Discharge_State = (uint8_t) ( ((_d[4] >> 5U) & (0x07U)) );
  _m->INV_Inverter_Command_Mode = (uint8_t) ( (_d[5] & (0x01U)) );
  _m->INV_Rolling_Counter = (uint8_t) ( ((_d[5] >> 4U) & (0x0FU)) );
  _m->INV_Inverter_Enable_State = (uint8_t) ( (_d[6] & (0x01U)) );
  _m->INV_Burst_Model_Mode = (uint8_t) ( ((_d[6] >> 1U) & (0x01U)) );
  _m->INV_BMS_Limiting_Regen_Torque = (uint8_t) ( ((_d[6] >> 2U) & (0x01U)) );
  _m->INV_Key_Switch_Start_Status = (uint8_t) ( ((_d[6] >> 6U) & (0x01U)) );
  _m->INV_Inverter_Enable_Lockout = (uint8_t) ( ((_d[6] >> 7U) & (0x01U)) );
  _m->INV_Direction_Command = (uint8_t) ( (_d[7] & (0x01U)) );
  _m->INV_BMS_Active = (uint8_t) ( ((_d[7] >> 1U) & (0x01U)) );
  _m->INV_BMS_Limiting_Motor_Torque = (uint8_t) ( ((_d[7] >> 2U) & (0x01U)) );
  _m->INV_Limit_Max_Speed = (uint8_t) ( ((_d[7] >> 3U) & (0x01U)) );
  _m->INV_Limit_Hot_Spot = (uint8_t) ( ((_d[7] >> 4U) & (0x01U)) );
  _m->INV_Low_Speed_Limiting = (uint8_t) ( ((_d[7] >> 5U) & (0x01U)) );
  _m->INV_Limit_Coolant_Derating = (uint8_t) ( ((_d[7] >> 6U) & (0x01U)) );
  _m->INV_Limit_Stall_Burst_Model = (uint8_t) ( ((_d[7] >> 7U) & (0x01U)) );

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M170_Internal_States_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M170_Internal_States_imxxx(&_m->mon1, M170_Internal_States_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M170_Internal_States_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M170_Internal_States_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->INV_VSM_State & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->INV_PWM_Frequency & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Inverter_State & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->INV_Relay_1_Status & (0x01U)) | ((_m->INV_Relay_2_Status & (0x01U)) << 1U) | ((_m->INV_Relay_3_Status & (0x01U)) << 2U) | ((_m->INV_Relay_4_Status & (0x01U)) << 3U) | ((_m->INV_Relay_5_Status & (0x01U)) << 4U) | ((_m->INV_Relay_6_Status & (0x01U)) << 5U) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Inverter_Run_Mode & (0x01U)) | ((_m->INV_Self_Sensing_Assist_Enable & (0x01U)) << 1U) | ((_m->INV_Inverter_Discharge_State & (0x07U)) << 5U) );
  cframe->Data[5] |= (uint8_t) ( (_m->INV_Inverter_Command_Mode & (0x01U)) | ((_m->INV_Rolling_Counter & (0x0FU)) << 4U) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Inverter_Enable_State & (0x01U)) | ((_m->INV_Burst_Model_Mode & (0x01U)) << 1U) | ((_m->INV_BMS_Limiting_Regen_Torque & (0x01U)) << 2U) | ((_m->INV_Key_Switch_Start_Status & (0x01U)) << 6U) | ((_m->INV_Inverter_Enable_Lockout & (0x01U)) << 7U) );
  cframe->Data[7] |= (uint8_t) ( (_m->INV_Direction_Command & (0x01U)) | ((_m->INV_BMS_Active & (0x01U)) << 1U) | ((_m->INV_BMS_Limiting_Motor_Torque & (0x01U)) << 2U) | ((_m->INV_Limit_Max_Speed & (0x01U)) << 3U) | ((_m->INV_Limit_Hot_Spot & (0x01U)) << 4U) | ((_m->INV_Low_Speed_Limiting & (0x01U)) << 5U) | ((_m->INV_Limit_Coolant_Derating & (0x01U)) << 6U) | ((_m->INV_Limit_Stall_Burst_Model & (0x01U)) << 7U) );

  cframe->MsgId = (uint32_t) M170_Internal_States_CANID;
  cframe->DLC = (uint8_t) M170_Internal_States_DLC;
  cframe->IDE = (uint8_t) M170_Internal_States_IDE;
  return M170_Internal_States_CANID;
}

#else

uint32_t Pack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M170_Internal_States_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->INV_VSM_State & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->INV_PWM_Frequency & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Inverter_State & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->INV_Relay_1_Status & (0x01U)) | ((_m->INV_Relay_2_Status & (0x01U)) << 1U) | ((_m->INV_Relay_3_Status & (0x01U)) << 2U) | ((_m->INV_Relay_4_Status & (0x01U)) << 3U) | ((_m->INV_Relay_5_Status & (0x01U)) << 4U) | ((_m->INV_Relay_6_Status & (0x01U)) << 5U) );
  _d[4] |= (uint8_t) ( (_m->INV_Inverter_Run_Mode & (0x01U)) | ((_m->INV_Self_Sensing_Assist_Enable & (0x01U)) << 1U) | ((_m->INV_Inverter_Discharge_State & (0x07U)) << 5U) );
  _d[5] |= (uint8_t) ( (_m->INV_Inverter_Command_Mode & (0x01U)) | ((_m->INV_Rolling_Counter & (0x0FU)) << 4U) );
  _d[6] |= (uint8_t) ( (_m->INV_Inverter_Enable_State & (0x01U)) | ((_m->INV_Burst_Model_Mode & (0x01U)) << 1U) | ((_m->INV_BMS_Limiting_Regen_Torque & (0x01U)) << 2U) | ((_m->INV_Key_Switch_Start_Status & (0x01U)) << 6U) | ((_m->INV_Inverter_Enable_Lockout & (0x01U)) << 7U) );
  _d[7] |= (uint8_t) ( (_m->INV_Direction_Command & (0x01U)) | ((_m->INV_BMS_Active & (0x01U)) << 1U) | ((_m->INV_BMS_Limiting_Motor_Torque & (0x01U)) << 2U) | ((_m->INV_Limit_Max_Speed & (0x01U)) << 3U) | ((_m->INV_Limit_Hot_Spot & (0x01U)) << 4U) | ((_m->INV_Low_Speed_Limiting & (0x01U)) << 5U) | ((_m->INV_Limit_Coolant_Derating & (0x01U)) << 6U) | ((_m->INV_Limit_Stall_Burst_Model & (0x01U)) << 7U) );

  *_len = (uint8_t) M170_Internal_States_DLC;
  *_ide = (uint8_t) M170_Internal_States_IDE;
  return M170_Internal_States_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Post_Fault_Lo = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->INV_Post_Fault_Hi = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->INV_Run_Fault_Lo = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->INV_Run_Fault_Hi = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M171_Fault_Codes_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M171_Fault_Codes_imxxx(&_m->mon1, M171_Fault_Codes_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M171_Fault_Codes_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M171_Fault_Codes_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Post_Fault_Lo & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Post_Fault_Lo >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Post_Fault_Hi & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Post_Fault_Hi >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Run_Fault_Lo & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Run_Fault_Lo >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Run_Fault_Hi & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Run_Fault_Hi >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M171_Fault_Codes_CANID;
  cframe->DLC = (uint8_t) M171_Fault_Codes_DLC;
  cframe->IDE = (uint8_t) M171_Fault_Codes_IDE;
  return M171_Fault_Codes_CANID;
}

#else

uint32_t Pack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M171_Fault_Codes_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->INV_Post_Fault_Lo & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Post_Fault_Lo >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Post_Fault_Hi & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Post_Fault_Hi >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Run_Fault_Lo & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Run_Fault_Lo >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Run_Fault_Hi & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Run_Fault_Hi >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M171_Fault_Codes_DLC;
  *_ide = (uint8_t) M171_Fault_Codes_IDE;
  return M171_Fault_Codes_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Commanded_Torque_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Commanded_Torque_phys = (sigfloat_t)(IMXXX_INV_Commanded_Torque_ro_fromS(_m->INV_Commanded_Torque_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Torque_Feedback_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Feedback_phys = (sigfloat_t)(IMXXX_INV_Torque_Feedback_ro_fromS(_m->INV_Torque_Feedback_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Power_On_Timer_ro = (uint32_t) ( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Power_On_Timer_phys = (sigfloat_t)(IMXXX_INV_Power_On_Timer_ro_fromS(_m->INV_Power_On_Timer_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M172_Torque_And_Timer_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M172_Torque_And_Timer_Info_imxxx(&_m->mon1, M172_Torque_And_Timer_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M172_Torque_And_Timer_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M172_Torque_And_Timer_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Commanded_Torque_ro = (int16_t) IMXXX_INV_Commanded_Torque_ro_toS(_m->INV_Commanded_Torque_phys);
  _m->INV_Torque_Feedback_ro = (int16_t) IMXXX_INV_Torque_Feedback_ro_toS(_m->INV_Torque_Feedback_phys);
  _m->INV_Power_On_Timer_ro = (uint32_t) IMXXX_INV_Power_On_Timer_ro_toS(_m->INV_Power_On_Timer_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Commanded_Torque_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Commanded_Torque_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Torque_Feedback_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Torque_Feedback_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Power_On_Timer_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M172_Torque_And_Timer_Info_CANID;
  cframe->DLC = (uint8_t) M172_Torque_And_Timer_Info_DLC;
  cframe->IDE = (uint8_t) M172_Torque_And_Timer_Info_IDE;
  return M172_Torque_And_Timer_Info_CANID;
}

#else

uint32_t Pack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M172_Torque_And_Timer_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Commanded_Torque_ro = (int16_t) IMXXX_INV_Commanded_Torque_ro_toS(_m->INV_Commanded_Torque_phys);
  _m->INV_Torque_Feedback_ro = (int16_t) IMXXX_INV_Torque_Feedback_ro_toS(_m->INV_Torque_Feedback_phys);
  _m->INV_Power_On_Timer_ro = (uint32_t) IMXXX_INV_Power_On_Timer_ro_toS(_m->INV_Power_On_Timer_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Commanded_Torque_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Commanded_Torque_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Torque_Feedback_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Torque_Feedback_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Power_On_Timer_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Power_On_Timer_ro >> 24U) & (0xFFU)) );

  *_len = (uint8_t) M172_Torque_And_Timer_Info_DLC;
  *_ide = (uint8_t) M172_Torque_And_Timer_Info_IDE;
  return M172_Torque_And_Timer_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Modulation_Index_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Modulation_Index_phys = (sigfloat_t)(IMXXX_INV_Modulation_Index_ro_fromS(_m->INV_Modulation_Index_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Flux_Weakening_Output_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Flux_Weakening_Output_phys = (sigfloat_t)(IMXXX_INV_Flux_Weakening_Output_ro_fromS(_m->INV_Flux_Weakening_Output_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Id_Command_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Id_Command_phys = (sigfloat_t)(IMXXX_INV_Id_Command_ro_fromS(_m->INV_Id_Command_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Iq_Command_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Iq_Command_phys = (sigfloat_t)(IMXXX_INV_Iq_Command_ro_fromS(_m->INV_Iq_Command_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M173_Modulation_And_Flux_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M173_Modulation_And_Flux_Info_imxxx(&_m->mon1, M173_Modulation_And_Flux_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M173_Modulation_And_Flux_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M173_Modulation_And_Flux_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Modulation_Index_ro = (int16_t) IMXXX_INV_Modulation_Index_ro_toS(_m->INV_Modulation_Index_phys);
  _m->INV_Flux_Weakening_Output_ro = (int16_t) IMXXX_INV_Flux_Weakening_Output_ro_toS(_m->INV_Flux_Weakening_Output_phys);
  _m->INV_Id_Command_ro = (int16_t) IMXXX_INV_Id_Command_ro_toS(_m->INV_Id_Command_phys);
  _m->INV_Iq_Command_ro = (int16_t) IMXXX_INV_Iq_Command_ro_toS(_m->INV_Iq_Command_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Modulation_Index_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Modulation_Index_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Flux_Weakening_Output_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Flux_Weakening_Output_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Id_Command_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Id_Command_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Iq_Command_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Iq_Command_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M173_Modulation_And_Flux_Info_CANID;
  cframe->DLC = (uint8_t) M173_Modulation_And_Flux_Info_DLC;
  cframe->IDE = (uint8_t) M173_Modulation_And_Flux_Info_IDE;
  return M173_Modulation_And_Flux_Info_CANID;
}

#else

uint32_t Pack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M173_Modulation_And_Flux_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Modulation_Index_ro = (int16_t) IMXXX_INV_Modulation_Index_ro_toS(_m->INV_Modulation_Index_phys);
  _m->INV_Flux_Weakening_Output_ro = (int16_t) IMXXX_INV_Flux_Weakening_Output_ro_toS(_m->INV_Flux_Weakening_Output_phys);
  _m->INV_Id_Command_ro = (int16_t) IMXXX_INV_Id_Command_ro_toS(_m->INV_Id_Command_phys);
  _m->INV_Iq_Command_ro = (int16_t) IMXXX_INV_Iq_Command_ro_toS(_m->INV_Iq_Command_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Modulation_Index_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Modulation_Index_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Flux_Weakening_Output_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Flux_Weakening_Output_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Id_Command_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Id_Command_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Iq_Command_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Iq_Command_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M173_Modulation_And_Flux_Info_DLC;
  *_ide = (uint8_t) M173_Modulation_And_Flux_Info_IDE;
  return M173_Modulation_And_Flux_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Project_Code_EEP_Ver = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->INV_SW_Version = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->INV_DateCode_MMDD = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->INV_DateCode_YYYY = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M174_Firmware_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M174_Firmware_Info_imxxx(&_m->mon1, M174_Firmware_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M174_Firmware_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M174_Firmware_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Project_Code_EEP_Ver & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Project_Code_EEP_Ver >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_SW_Version & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_SW_Version >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_DateCode_MMDD & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_DateCode_MMDD >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_DateCode_YYYY & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_DateCode_YYYY >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M174_Firmware_Info_CANID;
  cframe->DLC = (uint8_t) M174_Firmware_Info_DLC;
  cframe->IDE = (uint8_t) M174_Firmware_Info_IDE;
  return M174_Firmware_Info_CANID;
}

#else

uint32_t Pack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M174_Firmware_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->INV_Project_Code_EEP_Ver & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Project_Code_EEP_Ver >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_SW_Version & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_SW_Version >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_DateCode_MMDD & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_DateCode_MMDD >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_DateCode_YYYY & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_DateCode_YYYY >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M174_Firmware_Info_DLC;
  *_ide = (uint8_t) M174_Firmware_Info_IDE;
  return M174_Firmware_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Diag_Record = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->INV_Diag_Segment = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->INV_Diag_Gamma_Resolver_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Gamma_Resolver_phys = (sigfloat_t)(IMXXX_INV_Diag_Gamma_Resolver_ro_fromS(_m->INV_Diag_Gamma_Resolver_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Cos_Used = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->INV_Diag_Ic_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Ic_phys = (sigfloat_t)(IMXXX_INV_Diag_Ic_ro_fromS(_m->INV_Diag_Ic_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_PWM_Freq = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->INV_Diag_Id_cmd_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Id_cmd_phys = (sigfloat_t)(IMXXX_INV_Diag_Id_cmd_ro_fromS(_m->INV_Diag_Id_cmd_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Vq_Cmd_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Vq_Cmd_phys = (sigfloat_t)(IMXXX_INV_Diag_Vq_Cmd_ro_fromS(_m->INV_Diag_Vq_Cmd_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Gamma_Observer_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Gamma_Observer_phys = (sigfloat_t)(IMXXX_INV_Diag_Gamma_Observer_ro_fromS(_m->INV_Diag_Gamma_Observer_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Ia_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Ia_phys = (sigfloat_t)(IMXXX_INV_Diag_Ia_ro_fromS(_m->INV_Diag_Ia_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Run_Faults_Lo = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->INV_Diag_Vdc_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Vdc_phys = (sigfloat_t)(IMXXX_INV_Diag_Vdc_ro_fromS(_m->INV_Diag_Vdc_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Mod_Index_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Mod_Index_phys = (sigfloat_t)(IMXXX_INV_Diag_Mod_Index_ro_fromS(_m->INV_Diag_Mod_Index_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Vd_Cmd_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Vd_Cmd_phys = (sigfloat_t)(IMXXX_INV_Diag_Vd_Cmd_ro_fromS(_m->INV_Diag_Vd_Cmd_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Run_Faults_Hi = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
  _m->INV_Diag_Vqs_Cmd_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Vqs_Cmd_phys = (sigfloat_t)(IMXXX_INV_Diag_Vqs_Cmd_ro_fromS(_m->INV_Diag_Vqs_Cmd_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_FW_Output_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_FW_Output_phys = (sigfloat_t)(IMXXX_INV_Diag_FW_Output_ro_fromS(_m->INV_Diag_FW_Output_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Iq_cmd_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Iq_cmd_phys = (sigfloat_t)(IMXXX_INV_Diag_Iq_cmd_ro_fromS(_m->INV_Diag_Iq_cmd_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Ib_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Ib_phys = (sigfloat_t)(IMXXX_INV_Diag_Ib_ro_fromS(_m->INV_Diag_Ib_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Diag_Sin_Used = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M175_Diag_Data_Message_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M175_Diag_Data_Message_imxxx(&_m->mon1, M175_Diag_Data_Message_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M175_Diag_Data_Message_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M175_Diag_Data_Message_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Gamma_Resolver_ro = (int16_t) IMXXX_INV_Diag_Gamma_Resolver_ro_toS(_m->INV_Diag_Gamma_Resolver_phys);
  _m->INV_Diag_Ic_ro = (int16_t) IMXXX_INV_Diag_Ic_ro_toS(_m->INV_Diag_Ic_phys);
  _m->INV_Diag_Id_cmd_ro = (int16_t) IMXXX_INV_Diag_Id_cmd_ro_toS(_m->INV_Diag_Id_cmd_phys);
  _m->INV_Diag_Vq_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vq_Cmd_ro_toS(_m->INV_Diag_Vq_Cmd_phys);
  _m->INV_Diag_Gamma_Observer_ro = (int16_t) IMXXX_INV_Diag_Gamma_Observer_ro_toS(_m->INV_Diag_Gamma_Observer_phys);
  _m->INV_Diag_Ia_ro = (int16_t) IMXXX_INV_Diag_Ia_ro_toS(_m->INV_Diag_Ia_phys);
  _m->INV_Diag_Vdc_ro = (int16_t) IMXXX_INV_Diag_Vdc_ro_toS(_m->INV_Diag_Vdc_phys);
  _m->INV_Diag_Mod_Index_ro = (int16_t) IMXXX_INV_Diag_Mod_Index_ro_toS(_m->INV_Diag_Mod_Index_phys);
  _m->INV_Diag_Vd_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vd_Cmd_ro_toS(_m->INV_Diag_Vd_Cmd_phys);
  _m->INV_Diag_Vqs_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vqs_Cmd_ro_toS(_m->INV_Diag_Vqs_Cmd_phys);
  _m->INV_Diag_FW_Output_ro = (int16_t) IMXXX_INV_Diag_FW_Output_ro_toS(_m->INV_Diag_FW_Output_phys);
  _m->INV_Diag_Iq_cmd_ro = (int16_t) IMXXX_INV_Diag_Iq_cmd_ro_toS(_m->INV_Diag_Iq_cmd_phys);
  _m->INV_Diag_Ib_ro = (int16_t) IMXXX_INV_Diag_Ib_ro_toS(_m->INV_Diag_Ib_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Diag_Record & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->INV_Diag_Segment & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Diag_Gamma_Resolver_ro & (0xFFU)) | (_m->INV_Diag_Cos_Used & (0xFFU)) | (_m->INV_Diag_Ic_ro & (0xFFU)) | (_m->INV_Diag_PWM_Freq & (0xFFU)) | (_m->INV_Diag_Id_cmd_ro & (0xFFU)) | (_m->INV_Diag_Vq_Cmd_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Diag_Gamma_Resolver_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Cos_Used >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ic_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_PWM_Freq >> 8U) & (0xFFU)) | ((_m->INV_Diag_Id_cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vq_Cmd_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Diag_Gamma_Observer_ro & (0xFFU)) | (_m->INV_Diag_Ia_ro & (0xFFU)) | (_m->INV_Diag_Run_Faults_Lo & (0xFFU)) | (_m->INV_Diag_Vdc_ro & (0xFFU)) | (_m->INV_Diag_Mod_Index_ro & (0xFFU)) | (_m->INV_Diag_Vd_Cmd_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Diag_Gamma_Observer_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ia_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Run_Faults_Lo >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vdc_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Mod_Index_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vd_Cmd_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Diag_Run_Faults_Hi & (0xFFU)) | (_m->INV_Diag_Vqs_Cmd_ro & (0xFFU)) | (_m->INV_Diag_FW_Output_ro & (0xFFU)) | (_m->INV_Diag_Iq_cmd_ro & (0xFFU)) | (_m->INV_Diag_Ib_ro & (0xFFU)) | (_m->INV_Diag_Sin_Used & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Diag_Run_Faults_Hi >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vqs_Cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_FW_Output_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Iq_cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ib_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Sin_Used >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M175_Diag_Data_Message_CANID;
  cframe->DLC = (uint8_t) M175_Diag_Data_Message_DLC;
  cframe->IDE = (uint8_t) M175_Diag_Data_Message_IDE;
  return M175_Diag_Data_Message_CANID;
}

#else

uint32_t Pack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M175_Diag_Data_Message_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Diag_Gamma_Resolver_ro = (int16_t) IMXXX_INV_Diag_Gamma_Resolver_ro_toS(_m->INV_Diag_Gamma_Resolver_phys);
  _m->INV_Diag_Ic_ro = (int16_t) IMXXX_INV_Diag_Ic_ro_toS(_m->INV_Diag_Ic_phys);
  _m->INV_Diag_Id_cmd_ro = (int16_t) IMXXX_INV_Diag_Id_cmd_ro_toS(_m->INV_Diag_Id_cmd_phys);
  _m->INV_Diag_Vq_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vq_Cmd_ro_toS(_m->INV_Diag_Vq_Cmd_phys);
  _m->INV_Diag_Gamma_Observer_ro = (int16_t) IMXXX_INV_Diag_Gamma_Observer_ro_toS(_m->INV_Diag_Gamma_Observer_phys);
  _m->INV_Diag_Ia_ro = (int16_t) IMXXX_INV_Diag_Ia_ro_toS(_m->INV_Diag_Ia_phys);
  _m->INV_Diag_Vdc_ro = (int16_t) IMXXX_INV_Diag_Vdc_ro_toS(_m->INV_Diag_Vdc_phys);
  _m->INV_Diag_Mod_Index_ro = (int16_t) IMXXX_INV_Diag_Mod_Index_ro_toS(_m->INV_Diag_Mod_Index_phys);
  _m->INV_Diag_Vd_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vd_Cmd_ro_toS(_m->INV_Diag_Vd_Cmd_phys);
  _m->INV_Diag_Vqs_Cmd_ro = (int16_t) IMXXX_INV_Diag_Vqs_Cmd_ro_toS(_m->INV_Diag_Vqs_Cmd_phys);
  _m->INV_Diag_FW_Output_ro = (int16_t) IMXXX_INV_Diag_FW_Output_ro_toS(_m->INV_Diag_FW_Output_phys);
  _m->INV_Diag_Iq_cmd_ro = (int16_t) IMXXX_INV_Diag_Iq_cmd_ro_toS(_m->INV_Diag_Iq_cmd_phys);
  _m->INV_Diag_Ib_ro = (int16_t) IMXXX_INV_Diag_Ib_ro_toS(_m->INV_Diag_Ib_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Diag_Record & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->INV_Diag_Segment & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Diag_Gamma_Resolver_ro & (0xFFU)) | (_m->INV_Diag_Cos_Used & (0xFFU)) | (_m->INV_Diag_Ic_ro & (0xFFU)) | (_m->INV_Diag_PWM_Freq & (0xFFU)) | (_m->INV_Diag_Id_cmd_ro & (0xFFU)) | (_m->INV_Diag_Vq_Cmd_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Diag_Gamma_Resolver_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Cos_Used >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ic_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_PWM_Freq >> 8U) & (0xFFU)) | ((_m->INV_Diag_Id_cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vq_Cmd_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Diag_Gamma_Observer_ro & (0xFFU)) | (_m->INV_Diag_Ia_ro & (0xFFU)) | (_m->INV_Diag_Run_Faults_Lo & (0xFFU)) | (_m->INV_Diag_Vdc_ro & (0xFFU)) | (_m->INV_Diag_Mod_Index_ro & (0xFFU)) | (_m->INV_Diag_Vd_Cmd_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Diag_Gamma_Observer_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ia_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Run_Faults_Lo >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vdc_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Mod_Index_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vd_Cmd_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Diag_Run_Faults_Hi & (0xFFU)) | (_m->INV_Diag_Vqs_Cmd_ro & (0xFFU)) | (_m->INV_Diag_FW_Output_ro & (0xFFU)) | (_m->INV_Diag_Iq_cmd_ro & (0xFFU)) | (_m->INV_Diag_Ib_ro & (0xFFU)) | (_m->INV_Diag_Sin_Used & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Diag_Run_Faults_Hi >> 8U) & (0xFFU)) | ((_m->INV_Diag_Vqs_Cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_FW_Output_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Iq_cmd_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Ib_ro >> 8U) & (0xFFU)) | ((_m->INV_Diag_Sin_Used >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M175_Diag_Data_Message_DLC;
  *_ide = (uint8_t) M175_Diag_Data_Message_IDE;
  return M175_Diag_Data_Message_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Fast_Torque_Command_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Fast_Torque_Command_phys = (sigfloat_t)(IMXXX_INV_Fast_Torque_Command_ro_fromS(_m->INV_Fast_Torque_Command_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Fast_Torque_Feedback_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Fast_Torque_Feedback_phys = (sigfloat_t)(IMXXX_INV_Fast_Torque_Feedback_ro_fromS(_m->INV_Fast_Torque_Feedback_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Fast_Motor_Speed = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
  _m->INV_Fast_DC_Bus_Voltage_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Fast_DC_Bus_Voltage_phys = (sigfloat_t)(IMXXX_INV_Fast_DC_Bus_Voltage_ro_fromS(_m->INV_Fast_DC_Bus_Voltage_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M176_Fast_Info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M176_Fast_Info_imxxx(&_m->mon1, M176_Fast_Info_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M176_Fast_Info_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M176_Fast_Info_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Fast_Torque_Command_ro = (int16_t) IMXXX_INV_Fast_Torque_Command_ro_toS(_m->INV_Fast_Torque_Command_phys);
  _m->INV_Fast_Torque_Feedback_ro = (int16_t) IMXXX_INV_Fast_Torque_Feedback_ro_toS(_m->INV_Fast_Torque_Feedback_phys);
  _m->INV_Fast_DC_Bus_Voltage_ro = (int16_t) IMXXX_INV_Fast_DC_Bus_Voltage_ro_toS(_m->INV_Fast_DC_Bus_Voltage_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Fast_Torque_Command_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Fast_Torque_Command_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Fast_Torque_Feedback_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Fast_Torque_Feedback_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Fast_Motor_Speed & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Fast_Motor_Speed >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->INV_Fast_DC_Bus_Voltage_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->INV_Fast_DC_Bus_Voltage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M176_Fast_Info_CANID;
  cframe->DLC = (uint8_t) M176_Fast_Info_DLC;
  cframe->IDE = (uint8_t) M176_Fast_Info_IDE;
  return M176_Fast_Info_CANID;
}

#else

uint32_t Pack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M176_Fast_Info_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Fast_Torque_Command_ro = (int16_t) IMXXX_INV_Fast_Torque_Command_ro_toS(_m->INV_Fast_Torque_Command_phys);
  _m->INV_Fast_Torque_Feedback_ro = (int16_t) IMXXX_INV_Fast_Torque_Feedback_ro_toS(_m->INV_Fast_Torque_Feedback_phys);
  _m->INV_Fast_DC_Bus_Voltage_ro = (int16_t) IMXXX_INV_Fast_DC_Bus_Voltage_ro_toS(_m->INV_Fast_DC_Bus_Voltage_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Fast_Torque_Command_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Fast_Torque_Command_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Fast_Torque_Feedback_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Fast_Torque_Feedback_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->INV_Fast_Motor_Speed & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Fast_Motor_Speed >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->INV_Fast_DC_Bus_Voltage_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->INV_Fast_DC_Bus_Voltage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M176_Fast_Info_DLC;
  *_ide = (uint8_t) M176_Fast_Info_IDE;
  return M176_Fast_Info_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Torque_Capability_Motor_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Capability_Motor_phys = (sigfloat_t)(IMXXX_INV_Torque_Capability_Motor_ro_fromS(_m->INV_Torque_Capability_Motor_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->INV_Torque_Capability_Regen_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Capability_Regen_phys = (sigfloat_t)(IMXXX_INV_Torque_Capability_Regen_ro_fromS(_m->INV_Torque_Capability_Regen_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M177_Torque_Capability_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M177_Torque_Capability_imxxx(&_m->mon1, M177_Torque_Capability_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M177_Torque_Capability_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M177_Torque_Capability_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Capability_Motor_ro = (int16_t) IMXXX_INV_Torque_Capability_Motor_ro_toS(_m->INV_Torque_Capability_Motor_phys);
  _m->INV_Torque_Capability_Regen_ro = (int16_t) IMXXX_INV_Torque_Capability_Regen_ro_toS(_m->INV_Torque_Capability_Regen_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Torque_Capability_Motor_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Torque_Capability_Motor_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Torque_Capability_Regen_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->INV_Torque_Capability_Regen_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M177_Torque_Capability_CANID;
  cframe->DLC = (uint8_t) M177_Torque_Capability_DLC;
  cframe->IDE = (uint8_t) M177_Torque_Capability_IDE;
  return M177_Torque_Capability_CANID;
}

#else

uint32_t Pack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M177_Torque_Capability_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->INV_Torque_Capability_Motor_ro = (int16_t) IMXXX_INV_Torque_Capability_Motor_ro_toS(_m->INV_Torque_Capability_Motor_phys);
  _m->INV_Torque_Capability_Regen_ro = (int16_t) IMXXX_INV_Torque_Capability_Regen_ro_toS(_m->INV_Torque_Capability_Regen_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->INV_Torque_Capability_Motor_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Torque_Capability_Motor_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Torque_Capability_Regen_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->INV_Torque_Capability_Regen_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M177_Torque_Capability_DLC;
  *_ide = (uint8_t) M177_Torque_Capability_IDE;
  return M177_Torque_Capability_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VCU_INV_Torque_Command_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->VCU_INV_Torque_Command_phys = (sigfloat_t)(IMXXX_VCU_INV_Torque_Command_ro_fromS(_m->VCU_INV_Torque_Command_ro));
#endif // IMXXX_USE_SIGFLOAT

  _m->VCU_INV_Speed_Command = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->VCU_INV_Direction_Command = (uint8_t) ( (_d[4] & (0x01U)) );
  _m->VCU_INV_Inverter_Enable = (uint8_t) ( (_d[5] & (0x01U)) );
  _m->VCU_INV_Inverter_Discharge = (uint8_t) ( ((_d[5] >> 1U) & (0x01U)) );
  _m->VCU_INV_Speed_Mode_Enable = (uint8_t) ( ((_d[5] >> 2U) & (0x01U)) );
  _m->VCU_INV_Rolling_Counter = (uint8_t) ( ((_d[5] >> 4U) & (0x0FU)) );
  _m->VCU_INV_Torque_Limit_Command_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef IMXXX_USE_SIGFLOAT
  _m->VCU_INV_Torque_Limit_Command_phys = (sigfloat_t)(IMXXX_VCU_INV_Torque_Limit_Command_ro_fromS(_m->VCU_INV_Torque_Limit_Command_ro));
#endif // IMXXX_USE_SIGFLOAT

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M192_Command_Message_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M192_Command_Message_imxxx(&_m->mon1, M192_Command_Message_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M192_Command_Message_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M192_Command_Message_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->VCU_INV_Torque_Command_ro = (int16_t) IMXXX_VCU_INV_Torque_Command_ro_toS(_m->VCU_INV_Torque_Command_phys);
  _m->VCU_INV_Torque_Limit_Command_ro = (int16_t) IMXXX_VCU_INV_Torque_Limit_Command_ro_toS(_m->VCU_INV_Torque_Limit_Command_phys);
#endif // IMXXX_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->VCU_INV_Torque_Command_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->VCU_INV_Torque_Command_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->VCU_INV_Speed_Command & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->VCU_INV_Speed_Command >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->VCU_INV_Direction_Command & (0x01U)) );
  cframe->Data[5] |= (uint8_t) ( (_m->VCU_INV_Inverter_Enable & (0x01U)) | ((_m->VCU_INV_Inverter_Discharge & (0x01U)) << 1U) | ((_m->VCU_INV_Speed_Mode_Enable & (0x01U)) << 2U) | ((_m->VCU_INV_Rolling_Counter & (0x0FU)) << 4U) );
  cframe->Data[6] |= (uint8_t) ( (_m->VCU_INV_Torque_Limit_Command_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->VCU_INV_Torque_Limit_Command_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M192_Command_Message_CANID;
  cframe->DLC = (uint8_t) M192_Command_Message_DLC;
  cframe->IDE = (uint8_t) M192_Command_Message_IDE;
  return M192_Command_Message_CANID;
}

#else

uint32_t Pack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M192_Command_Message_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

#ifdef IMXXX_USE_SIGFLOAT
  _m->VCU_INV_Torque_Command_ro = (int16_t) IMXXX_VCU_INV_Torque_Command_ro_toS(_m->VCU_INV_Torque_Command_phys);
  _m->VCU_INV_Torque_Limit_Command_ro = (int16_t) IMXXX_VCU_INV_Torque_Limit_Command_ro_toS(_m->VCU_INV_Torque_Limit_Command_phys);
#endif // IMXXX_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->VCU_INV_Torque_Command_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->VCU_INV_Torque_Command_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->VCU_INV_Speed_Command & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->VCU_INV_Speed_Command >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->VCU_INV_Direction_Command & (0x01U)) );
  _d[5] |= (uint8_t) ( (_m->VCU_INV_Inverter_Enable & (0x01U)) | ((_m->VCU_INV_Inverter_Discharge & (0x01U)) << 1U) | ((_m->VCU_INV_Speed_Mode_Enable & (0x01U)) << 2U) | ((_m->VCU_INV_Rolling_Counter & (0x0FU)) << 4U) );
  _d[6] |= (uint8_t) ( (_m->VCU_INV_Torque_Limit_Command_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->VCU_INV_Torque_Limit_Command_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M192_Command_Message_DLC;
  *_ide = (uint8_t) M192_Command_Message_IDE;
  return M192_Command_Message_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->VCU_INV_Parameter_Address = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->VCU_INV_Parameter_RW_Command = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->VCU_INV_Parameter_Data = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M193_Read_Write_Param_Command_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M193_Read_Write_Param_Command_imxxx(&_m->mon1, M193_Read_Write_Param_Command_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M193_Read_Write_Param_Command_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M193_Read_Write_Param_Command_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->VCU_INV_Parameter_Address & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->VCU_INV_Parameter_Address >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->VCU_INV_Parameter_RW_Command & (0x01U)) );
  cframe->Data[4] |= (uint8_t) ( (_m->VCU_INV_Parameter_Data & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->VCU_INV_Parameter_Data >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M193_Read_Write_Param_Command_CANID;
  cframe->DLC = (uint8_t) M193_Read_Write_Param_Command_DLC;
  cframe->IDE = (uint8_t) M193_Read_Write_Param_Command_IDE;
  return M193_Read_Write_Param_Command_CANID;
}

#else

uint32_t Pack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M193_Read_Write_Param_Command_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->VCU_INV_Parameter_Address & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->VCU_INV_Parameter_Address >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->VCU_INV_Parameter_RW_Command & (0x01U)) );
  _d[4] |= (uint8_t) ( (_m->VCU_INV_Parameter_Data & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->VCU_INV_Parameter_Data >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M193_Read_Write_Param_Command_DLC;
  *_ide = (uint8_t) M193_Read_Write_Param_Command_IDE;
  return M193_Read_Write_Param_Command_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->INV_Parameter_Response_Addr = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->INV_Parameter_Response_Write_OK = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->INV_Parameter_Response_Data = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < M194_Read_Write_Param_Response_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_M194_Read_Write_Param_Response_imxxx(&_m->mon1, M194_Read_Write_Param_Response_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return M194_Read_Write_Param_Response_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M194_Read_Write_Param_Response_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->INV_Parameter_Response_Addr & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->INV_Parameter_Response_Addr >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->INV_Parameter_Response_Write_OK & (0x01U)) );
  cframe->Data[4] |= (uint8_t) ( (_m->INV_Parameter_Response_Data & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->INV_Parameter_Response_Data >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) M194_Read_Write_Param_Response_CANID;
  cframe->DLC = (uint8_t) M194_Read_Write_Param_Response_DLC;
  cframe->IDE = (uint8_t) M194_Read_Write_Param_Response_IDE;
  return M194_Read_Write_Param_Response_CANID;
}

#else

uint32_t Pack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(M194_Read_Write_Param_Response_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->INV_Parameter_Response_Addr & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->INV_Parameter_Response_Addr >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->INV_Parameter_Response_Write_OK & (0x01U)) );
  _d[4] |= (uint8_t) ( (_m->INV_Parameter_Response_Data & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->INV_Parameter_Response_Data >> 8U) & (0xFFU)) );

  *_len = (uint8_t) M194_Read_Write_Param_Response_DLC;
  *_ide = (uint8_t) M194_Read_Write_Param_Response_IDE;
  return M194_Read_Write_Param_Response_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->BMS_Max_Discharge_Current = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->BMS_Max_Charge_Current = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );

#ifdef IMXXX_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_Current_Limit_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_Current_Limit_imxxx(&_m->mon1, BMS_Current_Limit_CANID);
#endif // IMXXX_USE_DIAG_MONITORS

  return BMS_Current_Limit_CANID;
}

#ifdef IMXXX_USE_CANSTRUCT

uint32_t Pack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(BMS_Current_Limit_DLC); cframe->Data[i++] = IMXXX_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->BMS_Max_Discharge_Current & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->BMS_Max_Discharge_Current >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->BMS_Max_Charge_Current & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->BMS_Max_Charge_Current >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_Current_Limit_CANID;
  cframe->DLC = (uint8_t) BMS_Current_Limit_DLC;
  cframe->IDE = (uint8_t) BMS_Current_Limit_IDE;
  return BMS_Current_Limit_CANID;
}

#else

uint32_t Pack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < IMXXX_VALIDATE_DLC(BMS_Current_Limit_DLC); _d[i++] = IMXXX_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->BMS_Max_Discharge_Current & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->BMS_Max_Discharge_Current >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->BMS_Max_Charge_Current & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->BMS_Max_Charge_Current >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_Current_Limit_DLC;
  *_ide = (uint8_t) BMS_Current_Limit_IDE;
  return BMS_Current_Limit_CANID;
}

#endif // IMXXX_USE_CANSTRUCT

