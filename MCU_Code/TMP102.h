/*
 *  ======== TMP102.h ========
 *  TMP102 Interface
 */
#ifndef ti_sensors_TMP102__include
#define ti_sensors_TMP102__include 1


#define TMP102_ADDRESS 0x48
#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif

#define TMP102_TEMP 0x00
#define TMP102_CONFIG 0x01
#define TMP102_CONFIG_OS_ENABLE 0x8000
#define TMP102_CONFIG_OS_DISABLE 0x0000
#define TMP102_CONFIG_F_1FAULT 0x0000
#define TMP102_CONFIG_F_2FAULTS 0x0800
#define TMP102_CONFIG_F_4FAULTS 0x1000
#define TMP102_CONFIG_F_6FAULTS 0x1800
#define TMP102_CONFIG_POL_LOW 0x0000
#define TMP102_CONFIG_POL_HIGH 0x0400
#define TMP102_CONFIG_TM_COMP 0x0000
#define TMP102_CONFIG_TM_TSTAT 0x0200
#define TMP102_CONFIG_SD_CC 0x0000
#define TMP102_CONFIG_SD_SD 0x0100
#define TMP102_CONFIG_CR_0P25HZ 0x0000
#define TMP102_CONFIG_CR_1HZ 0x0040
#define TMP102_CONFIG_CR_4HZ 0x0080
#define TMP102_CONFIG_CR_8HZ 0x00C0
#define TMP102_CONFIG_AL_ENABLE 0x0020
#define TMP102_CONFIG_AL_DISABLE 0x0000
#define TMP102_CONFIG_EM_12BIT 0x0000
#define TMP102_CONFIG_EM_13BIT 0x0010
#define TMP102_TLOW 0x02
#define TMP102_THIGH 0x03

/*
 *  ======== TMP102_State ========
 *  Initial configuration state for a TMP102 sensor
 */
typedef struct TMP102_State {
    uint16_t config;
    uint16_t tlow;
    uint16_t thigh;

    uint8_t busId;   /* I2C bus id */
    uint8_t devAddr; /* Sensor's I2C address on the bus */

    uint16_t osWait; /* One shot conversion time (in ms)  */
} TMP102_State;

/*
 *  ======== TMP102_Handle ========
 *  First argument to all TMP102 methods
 */
typedef TMP102_State *TMP102_Handle;

/*
 *  ======== TMP102_config ========
 *  Configure device with current settings
 */
extern void TMP102_config(TMP102_Handle sensor);

/*
 *  ======== TMP102_read ========
 *  Read raw temperature register value
 *
 *  The returned value, when shifted right by a number of bits determined by
 *  the sensor's configuration register, represents the
 *  temperature in units of 0.0625 degrees Celsius.
 *
 *  Conversion functions are provided to convert the raw value returned by this
 *  function to a value in "natural" units.
 *
 *  For example, TMP102_toMilliCelsius(value) converts a raw temperature
 *  value into an integral temperature value in units of milli-degrees Celsius.
 *  
 *  @param sensor  handle to a TMP102 state object
 *  @returns       a 32-bit signed result in units of 0.0625 degrees C (when
 *                 shifted right by an appropriate amount)
 *
 *  @see TMP102_toIntCelsius TMP102_toFloatCelsius
 *       TMP102_toMilliCelsius
 */
extern int32_t TMP102_read(TMP102_Handle sensor);

/*
 *  ======== TMP102_toIntCelsius ========
 *  Convert raw temperature register value to whole degrees Celsius
 *
 *  This function rounds values to the nearest integer.
 *
 *  @param sensor pointer to the sensor's register state structure
 *  @param raw    32-bit sign extended temperature value read from the
 *                sensor's read function: TMP102_read()
 *  @returns      signed 32-bit integer representation of temperature in 
 *                degrees Celsius, rounded to the nearest whole number
 *  @see          TMP102_read
 */
extern int32_t TMP102_toIntCelsius(TMP102_Handle sensor, int32_t raw);

/*
 *  ======== TMP102_toMilliCelsius ========
 *  Convert raw field temperature to whole milli-degrees Celsius
 *
 *  This function truncates rather than rounds values.
 *
 *  @param sensor pointer to the sensor's register state structure
 *  @param raw    32-bit sign extended temperature value read from the
 *                sensor's read function: TMP102_read()
 *  @returns      signed 32-bit integer representation of temperature in 
 *                milli-degrees Celsius
 *  @see          TMP102_read
 */
extern int32_t TMP102_toMilliCelsius(TMP102_Handle sensor, int32_t raw);

/*
 *  ======== TMP102_toFloatCelsius ========
 *  Convert raw temperature value to fractional degrees Celsius
 *
 *  @param sensor pointer to the sensor's register state structure
 *  @param raw    32-bit sign extended temperature value read from the
 *                sensor's read function: TMP102_read()
 *  @returns      floating point representation of temperature in degrees
 *                Celsius
 *  @see          TMP102_read
 */
// extern float TMP102_toFloatCelsius(TMP102_Handle sensor, int32_t raw);

/* support C++ sources */
#ifdef __cplusplus
}
#endif

#endif

