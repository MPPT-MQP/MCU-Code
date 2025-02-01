//**********************************************************************************
//
// Custom Register definitions
//
//**********************************************************************************
#define EXT_ADC_ADDDRESS 0x48
#define CONVFACTOR 125 //(uV/ LSB)




/*Below values are 16 bits, need 8 bits for pico SDK*/

/* Register 0x00 (CONVERSION) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   Bit 15  |   Bit 14  |   Bit 13  |   Bit 12  |   Bit 11  |   Bit 10  |   Bit 9   |   Bit 8   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                           CONV[15:0]                                                                                          |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONVERSION register address */
    #define CONVERSION_ADDRESS                                              ((uint16_t) 0x00)

    /* CONVERSION default (reset) value */
    #define CONVERSION_DEFAULT                                              ((uint16_t) 0x0000)

    /* CONVERSION register field masks */
    #define CONVERSION_CONV_MASK                                            ((uint16_t) 0xFFFF)



/* Register 0x01 (CONFIG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   Bit 15  |   Bit 14  |   Bit 13  |   Bit 12  |   Bit 11  |   Bit 10  |   Bit 9   |   Bit 8   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     OS    |              MUX[2:0]             |              PGA[2:0]             |    MODE   |              DR[2:0]              | COMP_MODE |  COMP_POL |  COMP_LAT |     COMP_QUE[1:0]     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG register address */
    #define CONFIG_ADDRESS                                                  ((uint16_t) 0x01)

    /* CONFIG default (reset) value */
    #define CONFIG_DEFAULT                                                  ((uint16_t) 0x8583)

    /* CONFIG register field masks */
    #define CONFIG_OS_MASK                                                  ((uint16_t) 0x8000)
    #define CONFIG_MUX_MASK                                                 ((uint16_t) 0x7000)
    #define CONFIG_PGA_MASK                                                 ((uint16_t) 0x0E00)
    #define CONFIG_MODE_MASK                                                ((uint16_t) 0x0100)
    #define CONFIG_DR_MASK                                                  ((uint16_t) 0x00E0)
    #define CONFIG_COMP_MODE_MASK                                           ((uint16_t) 0x0010)
    #define CONFIG_COMP_POL_MASK                                            ((uint16_t) 0x0008)
    #define CONFIG_COMP_LAT_MASK                                            ((uint16_t) 0x0004)
    #define CONFIG_COMP_QUE_MASK                                            ((uint16_t) 0x0003)

    /* OS field values */
    #define CONFIG_OS_CONVERTING                                            ((uint16_t) 0x0000)
    #define CONFIG_OS_CONV_START                                            ((uint16_t) 0x8000)

    /* MUX field values */
    #define CONFIG_MUX_AIN0_AIN1                                            ((uint16_t) 0x0000)
    #define CONFIG_MUX_AIN0_AIN3                                            ((uint16_t) 0x1000)
    #define CONFIG_MUX_AIN1_AIN3                                            ((uint16_t) 0x2000)
    #define CONFIG_MUX_AIN2_AIN3                                            ((uint16_t) 0x3000)
    #define CONFIG_MUX_AIN0_GND                                             ((uint16_t) 0x4000)
    #define CONFIG_MUX_AIN1_GND                                             ((uint16_t) 0x5000)
    #define CONFIG_MUX_AIN2_GND                                             ((uint16_t) 0x6000)
    #define CONFIG_MUX_AIN3_GND                                             ((uint16_t) 0x7000)

    /* PGA field values */
    #define CONFIG_PGA_6p144V                                               ((uint16_t) 0x0000)
    #define CONFIG_PGA_4p096V                                               ((uint16_t) 0x0200)
    #define CONFIG_PGA_2p048V                                               ((uint16_t) 0x0400)
    #define CONFIG_PGA_1p024V                                               ((uint16_t) 0x0600)
    #define CONFIG_PGA_0p512V                                               ((uint16_t) 0x0800)
    #define CONFIG_PGA_0p256V                                               ((uint16_t) 0x0A00)
//    #define CONFIG_PGA_                                                     ((uint16_t) 0x0E00)

    /* MODE field values */
    #define CONFIG_MODE_CONT                                                ((uint16_t) 0x0000)
    #define CONFIG_MODE_SS                                                  ((uint16_t) 0x0100)

    /* DR field values */
    #define CONFIG_DR_8SPS                                                  ((uint16_t) 0x0000)
    #define CONFIG_DR_16SPS                                                 ((uint16_t) 0x0020)
    #define CONFIG_DR_32SPS                                                 ((uint16_t) 0x0040)
    #define CONFIG_DR_64SPS                                                 ((uint16_t) 0x0060)
    #define CONFIG_DR_128SPS                                                ((uint16_t) 0x0080)
    #define CONFIG_DR_250SPS                                                ((uint16_t) 0x00A0)
    #define CONFIG_DR_475SPS                                                ((uint16_t) 0x00C0)
    #define CONFIG_DR_860SPS                                                ((uint16_t) 0x00E0)

    /* COMP_MODE field values */
    #define CONFIG_COMP_MODE_TRAD                                           ((uint16_t) 0x0000)
    #define CONFIG_COMP_MODE_WINDOW                                         ((uint16_t) 0x0010)

    /* COMP_POL field values */
    #define CONFIG_COMP_POL_ACTIVE_LO                                       ((uint16_t) 0x0000)
    #define CONFIG_COMP_POL_ACTIVE_HI                                       ((uint16_t) 0x0008)

    /* COMP_LAT field values */
    #define CONFIG_COMP_LAT_NON_LATCH                                       ((uint16_t) 0x0000)
    #define CONFIG_COMP_LAT_LATCH                                           ((uint16_t) 0x0004)

    /* COMP_QUE field values */
    #define CONFIG_COMP_QUE_ASSERT1                                         ((uint16_t) 0x0000)
    #define CONFIG_COMP_QUE_ASSERT2                                         ((uint16_t) 0x0001)
    #define CONFIG_COMP_QUE_ASSERT4                                         ((uint16_t) 0x0002)
    #define CONFIG_COMP_QUE_DISABLE                                         ((uint16_t) 0x0003)



/* Register 0x02 (LO_THRESH) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   Bit 15  |   Bit 14  |   Bit 13  |   Bit 12  |   Bit 11  |   Bit 10  |   Bit 9   |   Bit 8   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                       LO_THRESHOLD[15:0]                                                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LO_THRESH register address */
    #define LO_THRESH_ADDRESS                                               ((uint16_t) 0x02)

    /* LO_THRESH default (reset) value */
    #define LO_THRESH_DEFAULT                                               ((uint16_t) 0x8000)

    /* LO_THRESH register field masks */
    #define LO_THRESH_LO_THRESHOLD_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x03 (HI_THRESH) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   Bit 15  |   Bit 14  |   Bit 13  |   Bit 12  |   Bit 11  |   Bit 10  |   Bit 9   |   Bit 8   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                       HI_THRESHOLD[15:0]                                                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HI_THRESH register address */
    #define HI_THRESH_ADDRESS                                               ((uint16_t) 0x03)

    /* HI_THRESH default (reset) value */
    #define HI_THRESH_DEFAULT                                               ((uint16_t) 0x7FFF)

    /* HI_THRESH register field masks */
    #define HI_THRESH_HI_THRESHOLD_MASK                                     ((uint16_t) 0xFFFF)


