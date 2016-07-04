/**
 * @file
 * @brief Dynamixel Device Library
 */
#define __STDC_LIMIT_MACROS
#include "data_serialization.h"
#include "variable.h"

#ifndef SerialDXL_h
#define SerialDXL_h
//------------------------------------------------------------------------------
/**
 * Debug macros
 */
//#define DEBUG_SERIAL_DXL
#ifdef DEBUG_SERIAL_DXL
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif
//------------------------------------------------------------------------------

#endif