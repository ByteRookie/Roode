/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */
 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "vl53l1_error_codes.h"
#include "VL53L1X_api.h"
#include "esphome/core/hal.h"
// Bring ESPHome HAL functions into global scope so third-party VL53L1X
// library files (which use bare millis/delay) compile without Arduino.h.
#ifdef __cplusplus
using esphome::millis;
using esphome::delay;
using esphome::delayMicroseconds;
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata,
        uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata,
        uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
        uint16_t      deviceAddress,
        uint16_t      registerAddress,
        uint32_t     *pdata);

#ifdef __cplusplus
}
#endif

#endif
