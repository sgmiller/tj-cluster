// Generator version : v3.1
// Generation time   : 2025.11.20 11:49:41
// DBC filename      : 20241119 iM-XXX (oil) CAN DB.dbc
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_IMXXX_MAJ (0U)
#define VER_IMXXX_MIN (0U)

// include current dbc-driver compilation config
#include "imxxx-config.h"

#ifdef IMXXX_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
#include "canmonitorutil.h"

#endif // IMXXX_USE_DIAG_MONITORS


// DLC maximum value which is used as the limit for frame's data buffer size.
// Client can set its own value (not sure why) in driver-config
// or can test it on some limit specified by application
// e.g.: static_assert(TESTDB_MAX_DLC_VALUE <= APPLICATION_FRAME_DATA_SIZE, "Max DLC value in the driver is too big")
#ifndef IMXXX_MAX_DLC_VALUE
// The value which was found out by generator (real max value)
#define IMXXX_MAX_DLC_VALUE 8U
#endif

// The limit is used for setting frame's data bytes
#define IMXXX_VALIDATE_DLC(msgDlc) (((msgDlc) <= (IMXXX_MAX_DLC_VALUE)) ? (msgDlc) : (IMXXX_MAX_DLC_VALUE))

// Initial byte value to be filles in data bytes of the frame before pack signals
// User can define its own custom value in driver-config file
#ifndef IMXXX_INITIAL_BYTE_VALUE
#define IMXXX_INITIAL_BYTE_VALUE 0U
#endif


// def @M160_Temperature_Set_1 CAN Message (160  0xa0)
#define M160_Temperature_Set_1_IDE (0U)
#define M160_Temperature_Set_1_DLC (8U)
#define M160_Temperature_Set_1_CANID (0xa0U)
#define M160_Temperature_Set_1_CYC (100U)
// signal: @INV_Module_A_Temp_ro
#define IMXXX_INV_Module_A_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Module_A_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Module_A_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Module_B_Temp_ro
#define IMXXX_INV_Module_B_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Module_B_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Module_B_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Module_C_Temp_ro
#define IMXXX_INV_Module_C_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Module_C_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Module_C_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Gate_Driver_Board_Temp_ro
#define IMXXX_INV_Gate_Driver_Board_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Gate_Driver_Board_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Gate_Driver_Board_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // IGBT Module A Temperature
  int16_t INV_Module_A_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_A_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // IGBT Module B Temperature
  int16_t INV_Module_B_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_B_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // IGBT Module C Temperature
  int16_t INV_Module_C_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_C_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Gate Driver Board Temperature
  int16_t INV_Gate_Driver_Board_Temp_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Gate_Driver_Board_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // IGBT Module A Temperature
  int16_t INV_Module_A_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_A_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // IGBT Module B Temperature
  int16_t INV_Module_B_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_B_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // IGBT Module C Temperature
  int16_t INV_Module_C_Temp_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Module_C_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Gate Driver Board Temperature
  int16_t INV_Gate_Driver_Board_Temp_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Gate_Driver_Board_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M160_Temperature_Set_1_t;

// def @M161_Temperature_Set_2 CAN Message (161  0xa1)
#define M161_Temperature_Set_2_IDE (0U)
#define M161_Temperature_Set_2_DLC (8U)
#define M161_Temperature_Set_2_CANID (0xa1U)
#define M161_Temperature_Set_2_CYC (100U)
// signal: @INV_Control_Board_Temp_ro
#define IMXXX_INV_Control_Board_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Control_Board_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Control_Board_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_RTD1_Temperature_ro
#define IMXXX_INV_RTD1_Temperature_ro_CovFactor (0.1)
#define IMXXX_INV_RTD1_Temperature_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_RTD1_Temperature_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_RTD2_Temperature_ro
#define IMXXX_INV_RTD2_Temperature_ro_CovFactor (0.1)
#define IMXXX_INV_RTD2_Temperature_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_RTD2_Temperature_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Stall_Burst_Model_Temp_ro
#define IMXXX_INV_Stall_Burst_Model_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Stall_Burst_Model_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Stall_Burst_Model_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Control Board Temperature
  int16_t INV_Control_Board_Temp_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Control_Board_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // RTD input 1 (PT1000) Temperature
  int16_t INV_RTD1_Temperature_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_RTD1_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // RTD input 2 (PT1000) Temperature
  int16_t INV_RTD2_Temperature_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_RTD2_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Hottest temperature estimated from the stall burst thermal model feature
  int16_t INV_Stall_Burst_Model_Temp_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Stall_Burst_Model_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // Control Board Temperature
  int16_t INV_Control_Board_Temp_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Control_Board_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // RTD input 1 (PT1000) Temperature
  int16_t INV_RTD1_Temperature_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_RTD1_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // RTD input 2 (PT1000) Temperature
  int16_t INV_RTD2_Temperature_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_RTD2_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Hottest temperature estimated from the stall burst thermal model feature
  int16_t INV_Stall_Burst_Model_Temp_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Stall_Burst_Model_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M161_Temperature_Set_2_t;

// def @M162_Temperature_Set_3 CAN Message (162  0xa2)
#define M162_Temperature_Set_3_IDE (0U)
#define M162_Temperature_Set_3_DLC (8U)
#define M162_Temperature_Set_3_CANID (0xa2U)
#define M162_Temperature_Set_3_CYC (100U)
// signal: @INV_Coolant_Temp_ro
#define IMXXX_INV_Coolant_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Coolant_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Coolant_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Hot_Spot_Temp_ro
#define IMXXX_INV_Hot_Spot_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Hot_Spot_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Hot_Spot_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Motor_Temp_ro
#define IMXXX_INV_Motor_Temp_ro_CovFactor (0.1)
#define IMXXX_INV_Motor_Temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Motor_Temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Torque_Shudder_ro
#define IMXXX_INV_Torque_Shudder_ro_CovFactor (0.1)
#define IMXXX_INV_Torque_Shudder_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Torque_Shudder_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Estimated Coolant Temperature
  int16_t INV_Coolant_Temp_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Coolant_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Estimated inverter hot spot temperature
  int16_t INV_Hot_Spot_Temp_ro;              //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Hot_Spot_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Motor Temperature Sensor
  int16_t INV_Motor_Temp_ro;                 //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Motor_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Shudder compensation value of torque
  int16_t INV_Torque_Shudder_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Shudder_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // Estimated Coolant Temperature
  int16_t INV_Coolant_Temp_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Coolant_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Estimated inverter hot spot temperature
  int16_t INV_Hot_Spot_Temp_ro;              //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Hot_Spot_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Motor Temperature Sensor
  int16_t INV_Motor_Temp_ro;                 //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Motor_Temp_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Shudder compensation value of torque
  int16_t INV_Torque_Shudder_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Shudder_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M162_Temperature_Set_3_t;

// def @M163_Analog_Input_Voltages CAN Message (163  0xa3)
#define M163_Analog_Input_Voltages_IDE (0U)
#define M163_Analog_Input_Voltages_DLC (8U)
#define M163_Analog_Input_Voltages_CANID (0xa3U)
#define M163_Analog_Input_Voltages_CYC (10U)
// signal: @INV_Oil_Temperature_ro
#define IMXXX_INV_Oil_Temperature_ro_CovFactor (0.1)
#define IMXXX_INV_Oil_Temperature_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Oil_Temperature_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Oil_Pressure_ro
#define IMXXX_INV_Oil_Pressure_ro_CovFactor (0.1)
#define IMXXX_INV_Oil_Pressure_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Oil_Pressure_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Analog_Input_4_ro
#define IMXXX_INV_Analog_Input_4_ro_CovFactor (0.01)
#define IMXXX_INV_Analog_Input_4_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Analog_Input_4_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @INV_Analog_Input_5_ro
#define IMXXX_INV_Analog_Input_5_ro_CovFactor (0.01)
#define IMXXX_INV_Analog_Input_5_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Analog_Input_5_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @INV_Analog_Input_6_ro
#define IMXXX_INV_Analog_Input_6_ro_CovFactor (0.01)
#define IMXXX_INV_Analog_Input_6_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Analog_Input_6_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Oil Temperature from sensor on motor assembly
  int16_t INV_Oil_Temperature_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Oil_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Oil Temperature from sensor on motor assembly
  int16_t INV_Oil_Pressure_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'pressure:psi'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Oil_Pressure_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #4
  uint16_t INV_Analog_Input_4_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_4_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #5
  uint16_t INV_Analog_Input_5_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #6
  uint16_t INV_Analog_Input_6_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_6_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // Oil Temperature from sensor on motor assembly
  int16_t INV_Oil_Temperature_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'temperature:C'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Oil_Temperature_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Oil Temperature from sensor on motor assembly
  int16_t INV_Oil_Pressure_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'pressure:psi'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Oil_Pressure_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #4
  uint16_t INV_Analog_Input_4_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_4_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #5
  uint16_t INV_Analog_Input_5_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Voltage on Analog Input #6
  uint16_t INV_Analog_Input_6_ro;            //      Bits=10 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Analog_Input_6_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M163_Analog_Input_Voltages_t;

// def @M164_Digital_Input_Status CAN Message (164  0xa4)
#define M164_Digital_Input_Status_IDE (0U)
#define M164_Digital_Input_Status_DLC (8U)
#define M164_Digital_Input_Status_CANID (0xa4U)
#define M164_Digital_Input_Status_CYC (10U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Status of Digital Input #1
  uint8_t INV_Digital_Input_1 : 1;           //      Bits= 1

  // Status of Digital Input #2
  uint8_t INV_Digital_Input_2 : 1;           //      Bits= 1

  // Status of Digital Input #3
  uint8_t INV_Digital_Input_3 : 1;           //      Bits= 1

  // Status of Digital Input #4
  uint8_t INV_Digital_Input_4 : 1;           //      Bits= 1

  // Status of Digital Input #5
  uint8_t INV_Digital_Input_5 : 1;           //      Bits= 1

  // Status of Digital Input #6
  uint8_t INV_Digital_Input_6 : 1;           //      Bits= 1

  // Status of Digital Input #7
  uint8_t INV_Digital_Input_7 : 1;           //      Bits= 1

  // Status of Digital Input #8
  uint8_t INV_Digital_Input_8 : 1;           //      Bits= 1

#else

  // Status of Digital Input #1
  uint8_t INV_Digital_Input_1;               //      Bits= 1

  // Status of Digital Input #2
  uint8_t INV_Digital_Input_2;               //      Bits= 1

  // Status of Digital Input #3
  uint8_t INV_Digital_Input_3;               //      Bits= 1

  // Status of Digital Input #4
  uint8_t INV_Digital_Input_4;               //      Bits= 1

  // Status of Digital Input #5
  uint8_t INV_Digital_Input_5;               //      Bits= 1

  // Status of Digital Input #6
  uint8_t INV_Digital_Input_6;               //      Bits= 1

  // Status of Digital Input #7
  uint8_t INV_Digital_Input_7;               //      Bits= 1

  // Status of Digital Input #8
  uint8_t INV_Digital_Input_8;               //      Bits= 1

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M164_Digital_Input_Status_t;

// def @M165_Motor_Position_Info CAN Message (165  0xa5)
#define M165_Motor_Position_Info_IDE (0U)
#define M165_Motor_Position_Info_DLC (8U)
#define M165_Motor_Position_Info_CANID (0xa5U)
#define M165_Motor_Position_Info_CYC (10U)
// signal: @INV_Motor_Angle_Electrical_ro
#define IMXXX_INV_Motor_Angle_Electrical_ro_CovFactor (0.1)
#define IMXXX_INV_Motor_Angle_Electrical_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Motor_Angle_Electrical_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Electrical_Output_Frequency_ro
#define IMXXX_INV_Electrical_Output_Frequency_ro_CovFactor (0.1)
#define IMXXX_INV_Electrical_Output_Frequency_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Electrical_Output_Frequency_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Delta_Resolver_Filtered_ro
#define IMXXX_INV_Delta_Resolver_Filtered_ro_CovFactor (0.1)
#define IMXXX_INV_Delta_Resolver_Filtered_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Delta_Resolver_Filtered_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The Electrical Angle of the motor as read by the encoder or resolver
  uint16_t INV_Motor_Angle_Electrical_ro;           //      Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Motor_Angle_Electrical_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured speed of the motor
  int16_t INV_Motor_Speed;                          //  [-] Bits=16 Unit:'angular_speed:rpm'

  // The actual electrical frequency of the inverter
  int16_t INV_Electrical_Output_Frequency_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'frequency:Hz'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Electrical_Output_Frequency_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Used in calibration of resolver angle adjustment.
  int16_t INV_Delta_Resolver_Filtered_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Delta_Resolver_Filtered_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The Electrical Angle of the motor as read by the encoder or resolver
  uint16_t INV_Motor_Angle_Electrical_ro;           //      Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Motor_Angle_Electrical_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured speed of the motor
  int16_t INV_Motor_Speed;                          //  [-] Bits=16 Unit:'angular_speed:rpm'

  // The actual electrical frequency of the inverter
  int16_t INV_Electrical_Output_Frequency_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'frequency:Hz'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Electrical_Output_Frequency_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Used in calibration of resolver angle adjustment.
  int16_t INV_Delta_Resolver_Filtered_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Delta_Resolver_Filtered_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M165_Motor_Position_Info_t;

// def @M166_Current_Info CAN Message (166  0xa6)
#define M166_Current_Info_IDE (0U)
#define M166_Current_Info_DLC (8U)
#define M166_Current_Info_CANID (0xa6U)
#define M166_Current_Info_CYC (10U)
// signal: @INV_Phase_A_Current_ro
#define IMXXX_INV_Phase_A_Current_ro_CovFactor (0.1)
#define IMXXX_INV_Phase_A_Current_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Phase_A_Current_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Phase_B_Current_ro
#define IMXXX_INV_Phase_B_Current_ro_CovFactor (0.1)
#define IMXXX_INV_Phase_B_Current_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Phase_B_Current_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Phase_C_Current_ro
#define IMXXX_INV_Phase_C_Current_ro_CovFactor (0.1)
#define IMXXX_INV_Phase_C_Current_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Phase_C_Current_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_DC_Bus_Current_ro
#define IMXXX_INV_DC_Bus_Current_ro_CovFactor (0.1)
#define IMXXX_INV_DC_Bus_Current_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_DC_Bus_Current_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The measured value of Phase A current
  int16_t INV_Phase_A_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_A_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured value of Phase B current
  int16_t INV_Phase_B_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_B_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured value of Phase C current
  int16_t INV_Phase_C_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_C_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The Calculated DC Bus Current
  int16_t INV_DC_Bus_Current_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_DC_Bus_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The measured value of Phase A current
  int16_t INV_Phase_A_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_A_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured value of Phase B current
  int16_t INV_Phase_B_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_B_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured value of Phase C current
  int16_t INV_Phase_C_Current_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Phase_C_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The Calculated DC Bus Current
  int16_t INV_DC_Bus_Current_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_DC_Bus_Current_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M166_Current_Info_t;

// def @M167_Voltage_Info CAN Message (167  0xa7)
#define M167_Voltage_Info_IDE (0U)
#define M167_Voltage_Info_DLC (8U)
#define M167_Voltage_Info_CANID (0xa7U)
#define M167_Voltage_Info_CYC (10U)
// signal: @INV_DC_Bus_Voltage_ro
#define IMXXX_INV_DC_Bus_Voltage_ro_CovFactor (0.1)
#define IMXXX_INV_DC_Bus_Voltage_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_DC_Bus_Voltage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Output_Voltage_ro
#define IMXXX_INV_Output_Voltage_ro_CovFactor (0.1)
#define IMXXX_INV_Output_Voltage_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Output_Voltage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_VAB_Vd_Voltage_ro
#define IMXXX_INV_VAB_Vd_Voltage_ro_CovFactor (0.1)
#define IMXXX_INV_VAB_Vd_Voltage_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_VAB_Vd_Voltage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_VBC_Vq_Voltage_ro
#define IMXXX_INV_VBC_Vq_Voltage_ro_CovFactor (0.1)
#define IMXXX_INV_VBC_Vq_Voltage_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_VBC_Vq_Voltage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The actual measured value of the DC bus voltage
  int16_t INV_DC_Bus_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_DC_Bus_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The calculated value of the output voltage, in peak line-neutral volts
  int16_t INV_Output_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Output_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Measured value of the voltage betwen phase A and Phase B
  int16_t INV_VAB_Vd_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_VAB_Vd_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Measured value of the voltage between Phase B and Phase C
  int16_t INV_VBC_Vq_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_VBC_Vq_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The actual measured value of the DC bus voltage
  int16_t INV_DC_Bus_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_DC_Bus_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The calculated value of the output voltage, in peak line-neutral volts
  int16_t INV_Output_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Output_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Measured value of the voltage betwen phase A and Phase B
  int16_t INV_VAB_Vd_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_VAB_Vd_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Measured value of the voltage between Phase B and Phase C
  int16_t INV_VBC_Vq_Voltage_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_VBC_Vq_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M167_Voltage_Info_t;

// def @M168_Flux_ID_IQ_Info CAN Message (168  0xa8)
#define M168_Flux_ID_IQ_Info_IDE (0U)
#define M168_Flux_ID_IQ_Info_DLC (8U)
#define M168_Flux_ID_IQ_Info_CANID (0xa8U)
#define M168_Flux_ID_IQ_Info_CYC (10U)
// signal: @INV_Vd_ff_ro
#define IMXXX_INV_Vd_ff_ro_CovFactor (0.1)
#define IMXXX_INV_Vd_ff_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Vd_ff_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Vq_ff_ro
#define IMXXX_INV_Vq_ff_ro_CovFactor (0.1)
#define IMXXX_INV_Vq_ff_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Vq_ff_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Id_ro
#define IMXXX_INV_Id_ro_CovFactor (0.1)
#define IMXXX_INV_Id_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Id_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Iq_ro
#define IMXXX_INV_Iq_ro_CovFactor (0.1)
#define IMXXX_INV_Iq_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Iq_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The d-axis voltage feedforward
  int16_t INV_Vd_ff_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Vd_ff_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The q-axis voltage feedforward
  int16_t INV_Vq_ff_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Vq_ff_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured Id current
  int16_t INV_Id_ro;                         //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Id_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured Iq current
  int16_t INV_Iq_ro;                         //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Iq_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The d-axis voltage feedforward
  int16_t INV_Vd_ff_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Vd_ff_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The q-axis voltage feedforward
  int16_t INV_Vq_ff_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Vq_ff_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured Id current
  int16_t INV_Id_ro;                         //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Id_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The measured Iq current
  int16_t INV_Iq_ro;                         //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Iq_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M168_Flux_ID_IQ_Info_t;

// def @M169_Internal_Voltages CAN Message (169  0xa9)
#define M169_Internal_Voltages_IDE (0U)
#define M169_Internal_Voltages_DLC (8U)
#define M169_Internal_Voltages_CANID (0xa9U)
#define M169_Internal_Voltages_CYC (100U)
// signal: @INV_Ref_Voltage_1_5_ro
#define IMXXX_INV_Ref_Voltage_1_5_ro_CovFactor (0.01)
#define IMXXX_INV_Ref_Voltage_1_5_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Ref_Voltage_1_5_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @INV_Ref_Voltage_2_5_ro
#define IMXXX_INV_Ref_Voltage_2_5_ro_CovFactor (0.01)
#define IMXXX_INV_Ref_Voltage_2_5_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Ref_Voltage_2_5_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @INV_Ref_Voltage_5_0_ro
#define IMXXX_INV_Ref_Voltage_5_0_ro_CovFactor (0.01)
#define IMXXX_INV_Ref_Voltage_5_0_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Ref_Voltage_5_0_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @INV_Ref_Voltage_12_0_ro
#define IMXXX_INV_Ref_Voltage_12_0_ro_CovFactor (0.01)
#define IMXXX_INV_Ref_Voltage_12_0_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define IMXXX_INV_Ref_Voltage_12_0_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Internal reference voltage
  int16_t INV_Ref_Voltage_1_5_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_1_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Internal reference voltage
  int16_t INV_Ref_Voltage_2_5_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_2_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Transducer voltage
  int16_t INV_Ref_Voltage_5_0_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_5_0_phys;
#endif // IMXXX_USE_SIGFLOAT

  // 12V Input Voltage
  int16_t INV_Ref_Voltage_12_0_ro;           //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_12_0_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // Internal reference voltage
  int16_t INV_Ref_Voltage_1_5_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_1_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Internal reference voltage
  int16_t INV_Ref_Voltage_2_5_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_2_5_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Transducer voltage
  int16_t INV_Ref_Voltage_5_0_ro;            //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_5_0_phys;
#endif // IMXXX_USE_SIGFLOAT

  // 12V Input Voltage
  int16_t INV_Ref_Voltage_12_0_ro;           //  [-] Bits=16 Factor= 0.01            Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Ref_Voltage_12_0_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M169_Internal_Voltages_t;

// def @M170_Internal_States CAN Message (170  0xaa)
#define M170_Internal_States_IDE (0U)
#define M170_Internal_States_DLC (8U)
#define M170_Internal_States_CANID (0xaaU)
#define M170_Internal_States_CYC (10U)

// Value tables for @INV_VSM_State signal

#ifndef INV_VSM_State_M170_Internal_States_VSM_Start_State
#define INV_VSM_State_M170_Internal_States_VSM_Start_State (0)
#endif

#ifndef INV_VSM_State_M170_Internal_States_PreCharge_Init_state
#define INV_VSM_State_M170_Internal_States_PreCharge_Init_state (1)
#endif

#ifndef INV_VSM_State_M170_Internal_States_precharge_active_state
#define INV_VSM_State_M170_Internal_States_precharge_active_state (2)
#endif

#ifndef INV_VSM_State_M170_Internal_States_precharge_complete_state
#define INV_VSM_State_M170_Internal_States_precharge_complete_state (3)
#endif

#ifndef INV_VSM_State_M170_Internal_States_VSM_wait_state
#define INV_VSM_State_M170_Internal_States_VSM_wait_state (4)
#endif

#ifndef INV_VSM_State_M170_Internal_States_VSM_ready_state
#define INV_VSM_State_M170_Internal_States_VSM_ready_state (5)
#endif

#ifndef INV_VSM_State_M170_Internal_States_Motor_Running_State
#define INV_VSM_State_M170_Internal_States_Motor_Running_State (6)
#endif

#ifndef INV_VSM_State_M170_Internal_States_blink_fault_code_state
#define INV_VSM_State_M170_Internal_States_blink_fault_code_state (7)
#endif

#ifndef INV_VSM_State_M170_Internal_States_Shutdown_state_for_Key_Switch_Mode_1
#define INV_VSM_State_M170_Internal_States_Shutdown_state_for_Key_Switch_Mode_1 (14)
#endif

#ifndef INV_VSM_State_M170_Internal_States_Reset_the_inverter
#define INV_VSM_State_M170_Internal_States_Reset_the_inverter (15)
#endif


// Value tables for @INV_Inverter_State signal

#ifndef INV_Inverter_State_M170_Internal_States_Power_up
#define INV_Inverter_State_M170_Internal_States_Power_up (0)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Stop
#define INV_Inverter_State_M170_Internal_States_Stop (1)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Open_Loop
#define INV_Inverter_State_M170_Internal_States_Open_Loop (2)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Closed_Loop
#define INV_Inverter_State_M170_Internal_States_Closed_Loop (3)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (4)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (5)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (6)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (7)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Idle_Run
#define INV_Inverter_State_M170_Internal_States_Idle_Run (8)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Idle_Stop
#define INV_Inverter_State_M170_Internal_States_Idle_Stop (9)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (10)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Intern_State
#define INV_Inverter_State_M170_Internal_States_Intern_State (11)
#endif

#ifndef INV_Inverter_State_M170_Internal_States_Internal_State
#define INV_Inverter_State_M170_Internal_States_Internal_State (12)
#endif


// Value tables for @INV_Inverter_Discharge_State signal

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Disabled
#define INV_Inverter_Discharge_State_M170_Internal_States_Disabled (0)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Enabled
#define INV_Inverter_Discharge_State_M170_Internal_States_Enabled (1)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Speed_Check
#define INV_Inverter_Discharge_State_M170_Internal_States_Speed_Check (2)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Active
#define INV_Inverter_Discharge_State_M170_Internal_States_Active (3)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Complete
#define INV_Inverter_Discharge_State_M170_Internal_States_Complete (4)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Error
#define INV_Inverter_Discharge_State_M170_Internal_States_Error (5)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Override
#define INV_Inverter_Discharge_State_M170_Internal_States_Override (6)
#endif

#ifndef INV_Inverter_Discharge_State_M170_Internal_States_Timeout
#define INV_Inverter_Discharge_State_M170_Internal_States_Timeout (7)
#endif


// Value tables for @INV_Burst_Model_Mode signal

#ifndef INV_Burst_Model_Mode_M170_Internal_States_Stall
#define INV_Burst_Model_Mode_M170_Internal_States_Stall (0)
#endif

#ifndef INV_Burst_Model_Mode_M170_Internal_States_High_Speed
#define INV_Burst_Model_Mode_M170_Internal_States_High_Speed (1)
#endif


// Value tables for @INV_Limit_Stall_Burst_Model signal

#ifndef INV_Limit_Stall_Burst_Model_M170_Internal_States_Not_Limiting
#define INV_Limit_Stall_Burst_Model_M170_Internal_States_Not_Limiting (0)
#endif

#ifndef INV_Limit_Stall_Burst_Model_M170_Internal_States_Limiting
#define INV_Limit_Stall_Burst_Model_M170_Internal_States_Limiting (1)
#endif


typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Different states for the vehicle state machine
  //  0 : "VSM Start State"
  //  1 : "Pre-Charge Init state"
  //  2 : "pre-charge active state"
  //  3 : "pre-charge complete state"
  //  4 : "VSM wait state"
  //  5 : "VSM ready state"
  //  6 : "Motor Running State"
  //  7 : "blink fault code state"
  //  14 : "Shutdown state for Key Switch Mode 1"
  //  15 : "Reset the inverter"
  uint8_t INV_VSM_State;                        //      Bits= 8

  // The current active PWM frequency
  uint8_t INV_PWM_Frequency;                    //      Bits= 8

  // Different states for the inverter state machine
  //  0 : "Power up"
  //  1 : "Stop"
  //  2 : "Open Loop"
  //  3 : "Closed Loop"
  //  4 : "Internal State"
  //  5 : "Internal State"
  //  6 : "Internal State"
  //  7 : "Internal State"
  //  8 : "Idle Run"
  //  9 : "Idle Stop"
  //  10 : "Internal State"
  //  11 : "Intern State"
  //  12 : "Internal State"
  uint8_t INV_Inverter_State;                   //      Bits= 8

  // 0=OFF, 1=ON
  uint8_t INV_Relay_1_Status : 1;               //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_2_Status : 1;               //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_3_Status : 1;               //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_4_Status : 1;               //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_5_Status : 1;               //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_6_Status : 1;               //      Bits= 1

  // 0=Torque Mode, 1=Speed Mode
  uint8_t INV_Inverter_Run_Mode : 1;            //      Bits= 1

  // 1 = Self Sensing Assist Enabled, 0 = Self Sensing Assist Disabled
  uint8_t INV_Self_Sensing_Assist_Enable : 1;   //      Bits= 1

  // 0 = Disabled, 1 = Enabled, 2 = Speed Check, 3 = Active, 4 = Complete, 5 = Error, 6 = Override, 7 = Timeout
  //  0 : "Disabled"
  //  1 : "Enabled"
  //  2 : "Speed Check"
  //  3 : "Active"
  //  4 : "Complete"
  //  5 : "Error"
  //  6 : "Override"
  //  7 : "Timeout"
  uint8_t INV_Inverter_Discharge_State : 3;     //      Bits= 3

  // 0=CAN mode, 1=VSM mode
  uint8_t INV_Inverter_Command_Mode : 1;        //      Bits= 1

  // Rolling Counter value
  uint8_t INV_Rolling_Counter : 4;              //      Bits= 4

  // 0=Inverter Disabled,
  // 1=Inverter Enabled
  uint8_t INV_Inverter_Enable_State : 1;        //      Bits= 1

  // 0 = Stall, 1 = High Speed
  //  0 : "Stall"
  //  1 : "High Speed"
  uint8_t INV_Burst_Model_Mode : 1;             //      Bits= 1

  // 0 = Not Limiting, 1 = Limiting
  uint8_t INV_BMS_Limiting_Regen_Torque : 1;    //      Bits= 1

  // 0 = OFF, 1 = ON
  uint8_t INV_Key_Switch_Start_Status : 1;      //      Bits= 1

  // 0=Lockout Disabled, 1=Lockout Enabled
  uint8_t INV_Inverter_Enable_Lockout : 1;      //      Bits= 1

  // 1 = Forward
  // 0 = 'Reverse' if inverter enabled  & 'Stopped' if inverter is disabled
  uint8_t INV_Direction_Command : 1;            //      Bits= 1

  // 0 = BMS Not Active, 1 = BMS Active
  uint8_t INV_BMS_Active : 1;                   //      Bits= 1

  // 0 = Not Limiting, 1 = Limiting
  uint8_t INV_BMS_Limiting_Motor_Torque : 1;    //      Bits= 1

  // Indicates that torque is being modified to limit the speed.
  uint8_t INV_Limit_Max_Speed : 1;              //      Bits= 1

  // Indicates that torque is being limited to limit the hot spot temp
  uint8_t INV_Limit_Hot_Spot : 1;               //      Bits= 1

  // Indicates that motor current is being limited due to low motor electrical frequency.
  uint8_t INV_Low_Speed_Limiting : 1;           //      Bits= 1

  // 0 = Not limiting, 1 = Limiting
  uint8_t INV_Limit_Coolant_Derating : 1;       //      Bits= 1

  // 0 = Not limiting, 1 = Limiting
  //  0 : "Not Limiting"
  //  1 : "Limiting"
  uint8_t INV_Limit_Stall_Burst_Model : 1;      //      Bits= 1

#else

  // Different states for the vehicle state machine
  //  0 : "VSM Start State"
  //  1 : "Pre-Charge Init state"
  //  2 : "pre-charge active state"
  //  3 : "pre-charge complete state"
  //  4 : "VSM wait state"
  //  5 : "VSM ready state"
  //  6 : "Motor Running State"
  //  7 : "blink fault code state"
  //  14 : "Shutdown state for Key Switch Mode 1"
  //  15 : "Reset the inverter"
  uint8_t INV_VSM_State;                        //      Bits= 8

  // The current active PWM frequency
  uint8_t INV_PWM_Frequency;                    //      Bits= 8

  // Different states for the inverter state machine
  //  0 : "Power up"
  //  1 : "Stop"
  //  2 : "Open Loop"
  //  3 : "Closed Loop"
  //  4 : "Internal State"
  //  5 : "Internal State"
  //  6 : "Internal State"
  //  7 : "Internal State"
  //  8 : "Idle Run"
  //  9 : "Idle Stop"
  //  10 : "Internal State"
  //  11 : "Intern State"
  //  12 : "Internal State"
  uint8_t INV_Inverter_State;                   //      Bits= 8

  // 0=OFF, 1=ON
  uint8_t INV_Relay_1_Status;                   //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_2_Status;                   //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_3_Status;                   //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_4_Status;                   //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_5_Status;                   //      Bits= 1

  // 0=OFF, 1=ON
  uint8_t INV_Relay_6_Status;                   //      Bits= 1

  // 0=Torque Mode, 1=Speed Mode
  uint8_t INV_Inverter_Run_Mode;                //      Bits= 1

  // 1 = Self Sensing Assist Enabled, 0 = Self Sensing Assist Disabled
  uint8_t INV_Self_Sensing_Assist_Enable;       //      Bits= 1

  // 0 = Disabled, 1 = Enabled, 2 = Speed Check, 3 = Active, 4 = Complete, 5 = Error, 6 = Override, 7 = Timeout
  //  0 : "Disabled"
  //  1 : "Enabled"
  //  2 : "Speed Check"
  //  3 : "Active"
  //  4 : "Complete"
  //  5 : "Error"
  //  6 : "Override"
  //  7 : "Timeout"
  uint8_t INV_Inverter_Discharge_State;         //      Bits= 3

  // 0=CAN mode, 1=VSM mode
  uint8_t INV_Inverter_Command_Mode;            //      Bits= 1

  // Rolling Counter value
  uint8_t INV_Rolling_Counter;                  //      Bits= 4

  // 0=Inverter Disabled,
  // 1=Inverter Enabled
  uint8_t INV_Inverter_Enable_State;            //      Bits= 1

  // 0 = Stall, 1 = High Speed
  //  0 : "Stall"
  //  1 : "High Speed"
  uint8_t INV_Burst_Model_Mode;                 //      Bits= 1

  // 0 = Not Limiting, 1 = Limiting
  uint8_t INV_BMS_Limiting_Regen_Torque;        //      Bits= 1

  // 0 = OFF, 1 = ON
  uint8_t INV_Key_Switch_Start_Status;          //      Bits= 1

  // 0=Lockout Disabled, 1=Lockout Enabled
  uint8_t INV_Inverter_Enable_Lockout;          //      Bits= 1

  // 1 = Forward
  // 0 = 'Reverse' if inverter enabled  & 'Stopped' if inverter is disabled
  uint8_t INV_Direction_Command;                //      Bits= 1

  // 0 = BMS Not Active, 1 = BMS Active
  uint8_t INV_BMS_Active;                       //      Bits= 1

  // 0 = Not Limiting, 1 = Limiting
  uint8_t INV_BMS_Limiting_Motor_Torque;        //      Bits= 1

  // Indicates that torque is being modified to limit the speed.
  uint8_t INV_Limit_Max_Speed;                  //      Bits= 1

  // Indicates that torque is being limited to limit the hot spot temp
  uint8_t INV_Limit_Hot_Spot;                   //      Bits= 1

  // Indicates that motor current is being limited due to low motor electrical frequency.
  uint8_t INV_Low_Speed_Limiting;               //      Bits= 1

  // 0 = Not limiting, 1 = Limiting
  uint8_t INV_Limit_Coolant_Derating;           //      Bits= 1

  // 0 = Not limiting, 1 = Limiting
  //  0 : "Not Limiting"
  //  1 : "Limiting"
  uint8_t INV_Limit_Stall_Burst_Model;          //      Bits= 1

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M170_Internal_States_t;

// def @M171_Fault_Codes CAN Message (171  0xab)
#define M171_Fault_Codes_IDE (0U)
#define M171_Fault_Codes_DLC (8U)
#define M171_Fault_Codes_CANID (0xabU)
#define M171_Fault_Codes_CYC (10U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Post_Fault_Lo;                //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Post_Fault_Hi;                //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Run_Fault_Lo;                 //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Run_Fault_Hi;                 //      Bits=16

#else

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Post_Fault_Lo;                //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Post_Fault_Hi;                //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Run_Fault_Lo;                 //      Bits=16

  // Each bit represents a fault. Please refer to PM100 Users Manual for details.
  uint16_t INV_Run_Fault_Hi;                 //      Bits=16

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M171_Fault_Codes_t;

// def @M172_Torque_And_Timer_Info CAN Message (172  0xac)
#define M172_Torque_And_Timer_Info_IDE (0U)
#define M172_Torque_And_Timer_Info_DLC (8U)
#define M172_Torque_And_Timer_Info_CANID (0xacU)
#define M172_Torque_And_Timer_Info_CYC (10U)
// signal: @INV_Commanded_Torque_ro
#define IMXXX_INV_Commanded_Torque_ro_CovFactor (0.1)
#define IMXXX_INV_Commanded_Torque_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Commanded_Torque_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Torque_Feedback_ro
#define IMXXX_INV_Torque_Feedback_ro_CovFactor (0.1)
#define IMXXX_INV_Torque_Feedback_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Torque_Feedback_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Power_On_Timer_ro
#define IMXXX_INV_Power_On_Timer_ro_CovFactor (0.003)
#define IMXXX_INV_Power_On_Timer_ro_toS(x) ( (uint32_t) (((x) - (0.0)) / (0.003)) )
#define IMXXX_INV_Power_On_Timer_ro_fromS(x) ( (((x) * (0.003)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The commanded Torque
  int16_t INV_Commanded_Torque_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Commanded_Torque_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Estimated motor torque feedback
  int16_t INV_Torque_Feedback_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Feedback_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Updated every 3 msec. This will roll over in approximately 150 days!
  uint32_t INV_Power_On_Timer_ro;            //      Bits=32 Factor= 0.003           Unit:'time:second'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Power_On_Timer_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The commanded Torque
  int16_t INV_Commanded_Torque_ro;           //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Commanded_Torque_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Estimated motor torque feedback
  int16_t INV_Torque_Feedback_ro;            //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Feedback_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Updated every 3 msec. This will roll over in approximately 150 days!
  uint32_t INV_Power_On_Timer_ro;            //      Bits=32 Factor= 0.003           Unit:'time:second'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Power_On_Timer_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M172_Torque_And_Timer_Info_t;

// def @M173_Modulation_And_Flux_Info CAN Message (173  0xad)
#define M173_Modulation_And_Flux_Info_IDE (0U)
#define M173_Modulation_And_Flux_Info_DLC (8U)
#define M173_Modulation_And_Flux_Info_CANID (0xadU)
#define M173_Modulation_And_Flux_Info_CYC (10U)
// signal: @INV_Modulation_Index_ro
#define IMXXX_INV_Modulation_Index_ro_CovFactor (0.0001)
#define IMXXX_INV_Modulation_Index_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define IMXXX_INV_Modulation_Index_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @INV_Flux_Weakening_Output_ro
#define IMXXX_INV_Flux_Weakening_Output_ro_CovFactor (0.1)
#define IMXXX_INV_Flux_Weakening_Output_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Flux_Weakening_Output_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Id_Command_ro
#define IMXXX_INV_Id_Command_ro_CovFactor (0.1)
#define IMXXX_INV_Id_Command_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Id_Command_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Iq_Command_ro
#define IMXXX_INV_Iq_Command_ro_CovFactor (0.1)
#define IMXXX_INV_Iq_Command_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Iq_Command_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // This is the modulation index. The scale factor is x100. To get the actual modulation index divide the value by 100.
  int16_t INV_Modulation_Index_ro;            //  [-] Bits=16 Factor= 0.0001

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Modulation_Index_phys;
#endif // IMXXX_USE_SIGFLOAT

  // This is the current output of the flux regulator.
  int16_t INV_Flux_Weakening_Output_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Flux_Weakening_Output_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The commanded D-axis current
  int16_t INV_Id_Command_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Id_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The commanded Q-axis current
  int16_t INV_Iq_Command_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Iq_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // This is the modulation index. The scale factor is x100. To get the actual modulation index divide the value by 100.
  int16_t INV_Modulation_Index_ro;            //  [-] Bits=16 Factor= 0.0001

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Modulation_Index_phys;
#endif // IMXXX_USE_SIGFLOAT

  // This is the current output of the flux regulator.
  int16_t INV_Flux_Weakening_Output_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Flux_Weakening_Output_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The commanded D-axis current
  int16_t INV_Id_Command_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Id_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The commanded Q-axis current
  int16_t INV_Iq_Command_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Iq_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M173_Modulation_And_Flux_Info_t;

// def @M174_Firmware_Info CAN Message (174  0xae)
#define M174_Firmware_Info_IDE (0U)
#define M174_Firmware_Info_DLC (8U)
#define M174_Firmware_Info_CANID (0xaeU)
#define M174_Firmware_Info_CYC (100U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  uint16_t INV_Project_Code_EEP_Ver;         //      Bits=16

  uint16_t INV_SW_Version;                   //      Bits=16

  uint16_t INV_DateCode_MMDD;                //      Bits=16

  uint16_t INV_DateCode_YYYY;                //      Bits=16

#else

  uint16_t INV_Project_Code_EEP_Ver;         //      Bits=16

  uint16_t INV_SW_Version;                   //      Bits=16

  uint16_t INV_DateCode_MMDD;                //      Bits=16

  uint16_t INV_DateCode_YYYY;                //      Bits=16

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M174_Firmware_Info_t;

// def @M175_Diag_Data_Message CAN Message (175  0xaf)
#define M175_Diag_Data_Message_IDE (0U)
#define M175_Diag_Data_Message_DLC (8U)
#define M175_Diag_Data_Message_CANID (0xafU)
#define M175_Diag_Data_Message_CYC (10U)
// signal: @INV_Diag_Gamma_Resolver_ro
#define IMXXX_INV_Diag_Gamma_Resolver_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Gamma_Resolver_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Gamma_Resolver_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Ic_ro
#define IMXXX_INV_Diag_Ic_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Ic_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Ic_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Id_cmd_ro
#define IMXXX_INV_Diag_Id_cmd_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Id_cmd_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Id_cmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Vq_Cmd_ro
#define IMXXX_INV_Diag_Vq_Cmd_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Vq_Cmd_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Vq_Cmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Gamma_Observer_ro
#define IMXXX_INV_Diag_Gamma_Observer_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Gamma_Observer_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Gamma_Observer_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Ia_ro
#define IMXXX_INV_Diag_Ia_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Ia_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Ia_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Vdc_ro
#define IMXXX_INV_Diag_Vdc_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Vdc_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Vdc_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Mod_Index_ro
#define IMXXX_INV_Diag_Mod_Index_ro_CovFactor (0.0001)
#define IMXXX_INV_Diag_Mod_Index_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define IMXXX_INV_Diag_Mod_Index_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @INV_Diag_Vd_Cmd_ro
#define IMXXX_INV_Diag_Vd_Cmd_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Vd_Cmd_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Vd_Cmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Vqs_Cmd_ro
#define IMXXX_INV_Diag_Vqs_Cmd_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Vqs_Cmd_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Vqs_Cmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_FW_Output_ro
#define IMXXX_INV_Diag_FW_Output_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_FW_Output_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_FW_Output_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Iq_cmd_ro
#define IMXXX_INV_Diag_Iq_cmd_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Iq_cmd_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Iq_cmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Diag_Ib_ro
#define IMXXX_INV_Diag_Ib_ro_CovFactor (0.1)
#define IMXXX_INV_Diag_Ib_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Diag_Ib_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  uint8_t INV_Diag_Record;                   //      Bits= 8

  // MULTIPLEX master signal
  uint8_t INV_Diag_Segment;                  //      Bits= 8

  // multiplex variable
  int16_t INV_Diag_Gamma_Resolver_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Gamma_Resolver_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Cos_Used;                 //  [-] Bits=16 Unit:'voltage:V'

  // multiplex variable
  int16_t INV_Diag_Ic_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ic_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_PWM_Freq;                //      Bits=16 Unit:'frequency:kHz'

  // multiplex variable
  int16_t INV_Diag_Id_cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Id_cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Vq_Cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vq_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Gamma_Observer_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Gamma_Observer_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Ia_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ia_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_Run_Faults_Lo;           //      Bits=16

  // multiplex variable
  int16_t INV_Diag_Vdc_ro;                   //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vdc_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Mod_Index_ro;             //  [-] Bits=16 Factor= 0.0001

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Mod_Index_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Vd_Cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vd_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_Run_Faults_Hi;           //      Bits=16

  // multiplex variable
  int16_t INV_Diag_Vqs_Cmd_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vqs_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_FW_Output_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_FW_Output_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Iq_cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Iq_cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Ib_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ib_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Sin_Used;                 //  [-] Bits=16 Unit:'voltage:V'

#else

  uint8_t INV_Diag_Record;                   //      Bits= 8

  // MULTIPLEX master signal
  uint8_t INV_Diag_Segment;                  //      Bits= 8

  // multiplex variable
  int16_t INV_Diag_Gamma_Resolver_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Gamma_Resolver_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Cos_Used;                 //  [-] Bits=16 Unit:'voltage:V'

  // multiplex variable
  int16_t INV_Diag_Ic_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ic_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_PWM_Freq;                //      Bits=16 Unit:'frequency:kHz'

  // multiplex variable
  int16_t INV_Diag_Id_cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Id_cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Vq_Cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vq_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Gamma_Observer_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'angle:deg'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Gamma_Observer_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Ia_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ia_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_Run_Faults_Lo;           //      Bits=16

  // multiplex variable
  int16_t INV_Diag_Vdc_ro;                   //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vdc_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Mod_Index_ro;             //  [-] Bits=16 Factor= 0.0001

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Mod_Index_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Vd_Cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vd_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  uint16_t INV_Diag_Run_Faults_Hi;           //      Bits=16

  // multiplex variable
  int16_t INV_Diag_Vqs_Cmd_ro;               //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Vqs_Cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_FW_Output_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_FW_Output_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Iq_cmd_ro;                //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Iq_cmd_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Ib_ro;                    //  [-] Bits=16 Factor= 0.1             Unit:'current:A'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Diag_Ib_phys;
#endif // IMXXX_USE_SIGFLOAT

  // multiplex variable
  int16_t INV_Diag_Sin_Used;                 //  [-] Bits=16 Unit:'voltage:V'

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M175_Diag_Data_Message_t;

// To enable fast message set CAN ACTIVE MSGS HI WORD to 0xFFFE.  Setting to default value of 0xFFFF will disable the fast message.
// def @M176_Fast_Info CAN Message (176  0xb0)
#define M176_Fast_Info_IDE (0U)
#define M176_Fast_Info_DLC (8U)
#define M176_Fast_Info_CANID (0xb0U)
#define M176_Fast_Info_CYC (3U)
// signal: @INV_Fast_Torque_Command_ro
#define IMXXX_INV_Fast_Torque_Command_ro_CovFactor (0.1)
#define IMXXX_INV_Fast_Torque_Command_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Fast_Torque_Command_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Fast_Torque_Feedback_ro
#define IMXXX_INV_Fast_Torque_Feedback_ro_CovFactor (0.1)
#define IMXXX_INV_Fast_Torque_Feedback_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Fast_Torque_Feedback_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Fast_DC_Bus_Voltage_ro
#define IMXXX_INV_Fast_DC_Bus_Voltage_ro_CovFactor (0.1)
#define IMXXX_INV_Fast_DC_Bus_Voltage_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Fast_DC_Bus_Voltage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The commanded torque
  int16_t INV_Fast_Torque_Command_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_Torque_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The estimated torque
  int16_t INV_Fast_Torque_Feedback_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_Torque_Feedback_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Motor speed
  int16_t INV_Fast_Motor_Speed;              //  [-] Bits=16 Unit:'angular_speed:rpm'

  // DC Bus Voltage
  int16_t INV_Fast_DC_Bus_Voltage_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_DC_Bus_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The commanded torque
  int16_t INV_Fast_Torque_Command_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_Torque_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The estimated torque
  int16_t INV_Fast_Torque_Feedback_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_Torque_Feedback_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Motor speed
  int16_t INV_Fast_Motor_Speed;              //  [-] Bits=16 Unit:'angular_speed:rpm'

  // DC Bus Voltage
  int16_t INV_Fast_DC_Bus_Voltage_ro;        //  [-] Bits=16 Factor= 0.1             Unit:'voltage:V'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Fast_DC_Bus_Voltage_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M176_Fast_Info_t;

// def @M177_Torque_Capability CAN Message (177  0xb1)
#define M177_Torque_Capability_IDE (0U)
#define M177_Torque_Capability_DLC (8U)
#define M177_Torque_Capability_CANID (0xb1U)
#define M177_Torque_Capability_CYC (10U)
// signal: @INV_Torque_Capability_Motor_ro
#define IMXXX_INV_Torque_Capability_Motor_ro_CovFactor (0.1)
#define IMXXX_INV_Torque_Capability_Motor_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Torque_Capability_Motor_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @INV_Torque_Capability_Regen_ro
#define IMXXX_INV_Torque_Capability_Regen_ro_CovFactor (0.1)
#define IMXXX_INV_Torque_Capability_Regen_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_INV_Torque_Capability_Regen_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // The motoring torque capability of the inverter given the current operating point.
  int16_t INV_Torque_Capability_Motor_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Capability_Motor_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The regen torque capability of the inverter given the current operating point.
  int16_t INV_Torque_Capability_Regen_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Capability_Regen_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // The motoring torque capability of the inverter given the current operating point.
  int16_t INV_Torque_Capability_Motor_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Capability_Motor_phys;
#endif // IMXXX_USE_SIGFLOAT

  // The regen torque capability of the inverter given the current operating point.
  int16_t INV_Torque_Capability_Regen_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t INV_Torque_Capability_Regen_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M177_Torque_Capability_t;

// The command message is used to transmit data to the controller. This message is sent from a user supplied external controller to the PMxxx controller.
// def @M192_Command_Message CAN Message (192  0xc0)
#define M192_Command_Message_IDE (0U)
#define M192_Command_Message_DLC (8U)
#define M192_Command_Message_CANID (0xc0U)
#define M192_Command_Message_CYC (10U)
// signal: @VCU_INV_Torque_Command_ro
#define IMXXX_VCU_INV_Torque_Command_ro_CovFactor (0.1)
#define IMXXX_VCU_INV_Torque_Command_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_VCU_INV_Torque_Command_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

// Value tables for @VCU_INV_Direction_Command signal

#ifndef VCU_INV_Direction_Command_M192_Command_Message_CW
#define VCU_INV_Direction_Command_M192_Command_Message_CW (0)
#endif

#ifndef VCU_INV_Direction_Command_M192_Command_Message_CCW
#define VCU_INV_Direction_Command_M192_Command_Message_CCW (1)
#endif


// Value tables for @VCU_INV_Inverter_Enable signal

#ifndef VCU_INV_Inverter_Enable_M192_Command_Message_Turn_the_inverter_OFF
#define VCU_INV_Inverter_Enable_M192_Command_Message_Turn_the_inverter_OFF (0)
#endif

#ifndef VCU_INV_Inverter_Enable_M192_Command_Message_Turn_the_Inverter_ON
#define VCU_INV_Inverter_Enable_M192_Command_Message_Turn_the_Inverter_ON (1)
#endif


// Value tables for @VCU_INV_Inverter_Discharge signal

#ifndef VCU_INV_Inverter_Discharge_M192_Command_Message_Discharge_Disable
#define VCU_INV_Inverter_Discharge_M192_Command_Message_Discharge_Disable (0)
#endif

#ifndef VCU_INV_Inverter_Discharge_M192_Command_Message_Discharge_Enable_if_EEPROM_parameter_is_set
#define VCU_INV_Inverter_Discharge_M192_Command_Message_Discharge_Enable_if_EEPROM_parameter_is_set (1)
#endif

// signal: @VCU_INV_Torque_Limit_Command_ro
#define IMXXX_VCU_INV_Torque_Limit_Command_ro_CovFactor (0.1)
#define IMXXX_VCU_INV_Torque_Limit_Command_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define IMXXX_VCU_INV_Torque_Limit_Command_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Torque command when in torque mode
  int16_t VCU_INV_Torque_Command_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t VCU_INV_Torque_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Speed command used when in speed mode
  int16_t VCU_INV_Speed_Command;                 //  [-] Bits=16 Unit:'angular_speed:rpm'

  // 0=Reverse, 1=Forward.  Forward is positive motor speed.
  //  0 : "CW"
  //  1 : "CCW"
  uint8_t VCU_INV_Direction_Command : 1;         //      Bits= 1

  // 0=Inverter OFF, 1 = Inverter ON
  //  0 : "Turn the inverter OFF"
  //  1 : "Turn the Inverter ON"
  uint8_t VCU_INV_Inverter_Enable : 1;           //      Bits= 1

  // 0=Discharge Disable,1=Discharge Enable
  //  0 : "Discharge Disable"
  //  1 : "Discharge Enable (if EEPROM parameter is set)"
  uint8_t VCU_INV_Inverter_Discharge : 1;        //      Bits= 1

  // 0 = No change to mode, 1 = change to speed mode from torque mode
  uint8_t VCU_INV_Speed_Mode_Enable : 1;         //      Bits= 1

  // Rolling Counter sent to inverter.  If used increment count with each message sent.  Otherwise can be set to 0.
  uint8_t VCU_INV_Rolling_Counter : 4;           //      Bits= 4

  // Torque Limit, set to 0 to keep default
  int16_t VCU_INV_Torque_Limit_Command_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t VCU_INV_Torque_Limit_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

#else

  // Torque command when in torque mode
  int16_t VCU_INV_Torque_Command_ro;             //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t VCU_INV_Torque_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

  // Speed command used when in speed mode
  int16_t VCU_INV_Speed_Command;                 //  [-] Bits=16 Unit:'angular_speed:rpm'

  // 0=Reverse, 1=Forward.  Forward is positive motor speed.
  //  0 : "CW"
  //  1 : "CCW"
  uint8_t VCU_INV_Direction_Command;             //      Bits= 1

  // 0=Inverter OFF, 1 = Inverter ON
  //  0 : "Turn the inverter OFF"
  //  1 : "Turn the Inverter ON"
  uint8_t VCU_INV_Inverter_Enable;               //      Bits= 1

  // 0=Discharge Disable,1=Discharge Enable
  //  0 : "Discharge Disable"
  //  1 : "Discharge Enable (if EEPROM parameter is set)"
  uint8_t VCU_INV_Inverter_Discharge;            //      Bits= 1

  // 0 = No change to mode, 1 = change to speed mode from torque mode
  uint8_t VCU_INV_Speed_Mode_Enable;             //      Bits= 1

  // Rolling Counter sent to inverter.  If used increment count with each message sent.  Otherwise can be set to 0.
  uint8_t VCU_INV_Rolling_Counter;               //      Bits= 4

  // Torque Limit, set to 0 to keep default
  int16_t VCU_INV_Torque_Limit_Command_ro;       //  [-] Bits=16 Factor= 0.1             Unit:'torque:N.m'

#ifdef IMXXX_USE_SIGFLOAT
  sigfloat_t VCU_INV_Torque_Limit_Command_phys;
#endif // IMXXX_USE_SIGFLOAT

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M192_Command_Message_t;

// To write a parameter use message 0x0C1 with byte #2 set to 1 (write).
// To read a parameter use message 0x0C1 with byte #2 to set 0 (read). 
// def @M193_Read_Write_Param_Command CAN Message (193  0xc1)
#define M193_Read_Write_Param_Command_IDE (0U)
#define M193_Read_Write_Param_Command_DLC (8U)
#define M193_Read_Write_Param_Command_CANID (0xc1U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Address of parameter to be written or read.
  uint16_t VCU_INV_Parameter_Address;         //      Bits=16

  // 0=Read, 1=Write
  uint8_t VCU_INV_Parameter_RW_Command : 1;   //      Bits= 1

  // Data to be written.  All data is 16 bits and is contained in bytes 4 and 5.
  int16_t VCU_INV_Parameter_Data;             //  [-] Bits=16

#else

  // Address of parameter to be written or read.
  uint16_t VCU_INV_Parameter_Address;         //      Bits=16

  // 0=Read, 1=Write
  uint8_t VCU_INV_Parameter_RW_Command;       //      Bits= 1

  // Data to be written.  All data is 16 bits and is contained in bytes 4 and 5.
  int16_t VCU_INV_Parameter_Data;             //  [-] Bits=16

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M193_Read_Write_Param_Command_t;

// To write a parameter use message 0x0C1 with byte #2 set to 1 (write).
// To read a parameter use message 0x0C1 with byte #2 to set 0 (read). 
// def @M194_Read_Write_Param_Response CAN Message (194  0xc2)
#define M194_Read_Write_Param_Response_IDE (0U)
#define M194_Read_Write_Param_Response_DLC (8U)
#define M194_Read_Write_Param_Response_CANID (0xc2U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Address of parameter response message data.
  uint16_t INV_Parameter_Response_Addr;          //      Bits=16

  // 0=Write failure, 1=Success
  uint8_t INV_Parameter_Response_Write_OK : 1;   //      Bits= 1

  // Data from parameter message.  All data is 16 bits and is contained in bytes 4 and 5. Bytes 6 and 7 should be ignored.
  int16_t INV_Parameter_Response_Data;           //  [-] Bits=16

#else

  // Address of parameter response message data.
  uint16_t INV_Parameter_Response_Addr;          //      Bits=16

  // 0=Write failure, 1=Success
  uint8_t INV_Parameter_Response_Write_OK;       //      Bits= 1

  // Data from parameter message.  All data is 16 bits and is contained in bytes 4 and 5. Bytes 6 and 7 should be ignored.
  int16_t INV_Parameter_Response_Data;           //  [-] Bits=16

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} M194_Read_Write_Param_Response_t;

// Sent by BMS
// def @BMS_Current_Limit CAN Message (514  0x202)
#define BMS_Current_Limit_IDE (0U)
#define BMS_Current_Limit_DLC (8U)
#define BMS_Current_Limit_CANID (0x202U)

typedef struct
{
#ifdef IMXXX_USE_BITS_SIGNAL

  // Maximum discharge current from BMS
  uint16_t BMS_Max_Discharge_Current;        //      Bits=16 Unit:'current:A'

  // Maximum charge current from BMS
  uint16_t BMS_Max_Charge_Current;           //      Bits=16 Unit:'current:A'

#else

  // Maximum discharge current from BMS
  uint16_t BMS_Max_Discharge_Current;        //      Bits=16 Unit:'current:A'

  // Maximum charge current from BMS
  uint16_t BMS_Max_Charge_Current;           //      Bits=16 Unit:'current:A'

#endif // IMXXX_USE_BITS_SIGNAL

#ifdef IMXXX_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // IMXXX_USE_DIAG_MONITORS

} BMS_Current_Limit_t;

// Function signatures

uint32_t Unpack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M160_Temperature_Set_1_imXXX(M160_Temperature_Set_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M161_Temperature_Set_2_imXXX(M161_Temperature_Set_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M162_Temperature_Set_3_imXXX(M162_Temperature_Set_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M163_Analog_Input_Voltages_imXXX(M163_Analog_Input_Voltages_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M164_Digital_Input_Status_imXXX(M164_Digital_Input_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M165_Motor_Position_Info_imXXX(M165_Motor_Position_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M166_Current_Info_imXXX(M166_Current_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M167_Voltage_Info_imXXX(M167_Voltage_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M168_Flux_ID_IQ_Info_imXXX(M168_Flux_ID_IQ_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M169_Internal_Voltages_imXXX(M169_Internal_Voltages_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M170_Internal_States_imXXX(M170_Internal_States_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M171_Fault_Codes_imXXX(M171_Fault_Codes_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M172_Torque_And_Timer_Info_imXXX(M172_Torque_And_Timer_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M173_Modulation_And_Flux_Info_imXXX(M173_Modulation_And_Flux_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M174_Firmware_Info_imXXX(M174_Firmware_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M175_Diag_Data_Message_imXXX(M175_Diag_Data_Message_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M176_Fast_Info_imXXX(M176_Fast_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M177_Torque_Capability_imXXX(M177_Torque_Capability_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M192_Command_Message_imXXX(M192_Command_Message_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M193_Read_Write_Param_Command_imXXX(M193_Read_Write_Param_Command_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_M194_Read_Write_Param_Response_imXXX(M194_Read_Write_Param_Response_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

uint32_t Unpack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef IMXXX_USE_CANSTRUCT
uint32_t Pack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_Current_Limit_imXXX(BMS_Current_Limit_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // IMXXX_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
