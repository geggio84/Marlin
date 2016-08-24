/**
 ******************************************************************************
 * @file    easyspin.h
 * @author  EMEA AMS-IPD Marketing & Application, Prague - VE
 * @version V1.0.1
 * @date    June-2012
 * @brief   Header for easyspin.c module
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EASYSPIN_H
#define __EASYSPIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pins.h"

extern uint32_t mode;
extern uint8_t bits;
extern uint32_t speed;
extern uint16_t delay;
extern int spi_fd;

/* Exported constants --------------------------------------------------------*/
#define easySPIN_STBY_RESET_GPIO STEPPER_ENABLEn_PIN
#define easySPIN_FLAG_GPIO STEPPER_FLAG_PIN

/** @defgroup easySPIN register bits / masks
 * @{
 */
/* easySPIN absolute position register masks */
#define easySPIN_ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)
#define easySPIN_ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)

/* easySPIN electrical position register masks */
#define easySPIN_ELPOS_STEP_MASK       ((uint8_t)0xC0)
#define easySPIN_ELPOS_MICROSTEP_MASK  ((uint8_t)0x3F)

/* Exported types ------------------------------------------------------------*/

/** 
 * @brief easySPIN Init structure definition
 */
typedef struct {
	uint32_t ABS_POS;
	uint16_t EL_POS;
	uint32_t MARK;
	uint8_t TVAL;
	uint8_t T_FAST;
	uint8_t TON_MIN;
	uint8_t TOFF_MIN;
	uint8_t ADC_OUT;
	uint8_t OCD_TH;
	uint8_t STEP_MODE;
	uint8_t ALARM_EN;
	uint16_t CONFIG;
} easySPIN_RegsStruct_TypeDef;

/* easySPIN T_FAST register options */
typedef enum {
	easySPIN_TOFF_FAST_2_us = ((uint8_t) 0x00 << 4),
	easySPIN_TOFF_FAST_4_us = ((uint8_t) 0x01 << 4),
	easySPIN_TOFF_FAST_6_us = ((uint8_t) 0x02 << 4),
	easySPIN_TOFF_FAST_8_us = ((uint8_t) 0x03 << 4),
	easySPIN_TOFF_FAST_10_us = ((uint8_t) 0x04 << 4),
	easySPIN_TOFF_FAST_12_us = ((uint8_t) 0x05 << 4),
	easySPIN_TOFF_FAST_14_us = ((uint8_t) 0x06 << 4),
	easySPIN_TOFF_FAST_16_us = ((uint8_t) 0x07 << 4),
	easySPIN_TOFF_FAST_18_us = ((uint8_t) 0x08 << 4),
	easySPIN_TOFF_FAST_20_us = ((uint8_t) 0x09 << 4),
	easySPIN_TOFF_FAST_22_us = ((uint8_t) 0x0A << 4),
	easySPIN_TOFF_FAST_24_us = ((uint8_t) 0x0B << 4),
	easySPIN_TOFF_FAST_26_us = ((uint8_t) 0x0C << 4),
	easySPIN_TOFF_FAST_28_us = ((uint8_t) 0x0D << 4),
	easySPIN_TOFF_FAST_30_us = ((uint8_t) 0x0E << 4),
	easySPIN_TOFF_FAST_32_us = ((uint8_t) 0x0F << 4)
} easySPIN_TOFF_FAST_TypeDef;

typedef enum {
	easySPIN_FAST_STEP_2_us  = ((uint8_t) 0x00),
	easySPIN_FAST_STEP_4_us  = ((uint8_t) 0x01),
	easySPIN_FAST_STEP_6_us  = ((uint8_t) 0x02),
	easySPIN_FAST_STEP_8_us  = ((uint8_t) 0x03),
	easySPIN_FAST_STEP_10_us = ((uint8_t) 0x04),
	easySPIN_FAST_STEP_12_us = ((uint8_t) 0x05),
	easySPIN_FAST_STEP_14_us = ((uint8_t) 0x06),
	easySPIN_FAST_STEP_16_us = ((uint8_t) 0x07),
	easySPIN_FAST_STEP_18_us = ((uint8_t) 0x08),
	easySPIN_FAST_STEP_20_us = ((uint8_t) 0x09),
	easySPIN_FAST_STEP_22_us = ((uint8_t) 0x0A),
	easySPIN_FAST_STEP_24_us = ((uint8_t) 0x0B),
	easySPIN_FAST_STEP_26_us = ((uint8_t) 0x0C),
	easySPIN_FAST_STEP_28_us = ((uint8_t) 0x0D),
	easySPIN_FAST_STEP_30_us = ((uint8_t) 0x0E),
	easySPIN_FAST_STEP_32_us = ((uint8_t) 0x0F)
} easySPIN_FAST_STEP_TypeDef;

/* easySPIN overcurrent threshold options */
typedef enum {
	easySPIN_OCD_TH_375mA  = ((uint8_t) 0x00),
	easySPIN_OCD_TH_750mA  = ((uint8_t) 0x01),
	easySPIN_OCD_TH_1125mA = ((uint8_t) 0x02),
	easySPIN_OCD_TH_1500mA = ((uint8_t) 0x03),
	easySPIN_OCD_TH_1875mA = ((uint8_t) 0x04),
	easySPIN_OCD_TH_2250mA = ((uint8_t) 0x05),
	easySPIN_OCD_TH_2625mA = ((uint8_t) 0x06),
	easySPIN_OCD_TH_3000mA = ((uint8_t) 0x07),
	easySPIN_OCD_TH_3375mA = ((uint8_t) 0x08),
	easySPIN_OCD_TH_3750mA = ((uint8_t) 0x09),
	easySPIN_OCD_TH_4125mA = ((uint8_t) 0x0A),
	easySPIN_OCD_TH_4500mA = ((uint8_t) 0x0B),
	easySPIN_OCD_TH_4875mA = ((uint8_t) 0x0C),
	easySPIN_OCD_TH_5250mA = ((uint8_t) 0x0D),
	easySPIN_OCD_TH_5625mA = ((uint8_t) 0x0E),
	easySPIN_OCD_TH_6000mA = ((uint8_t) 0x0F)
} easySPIN_OCD_TH_TypeDef;

/* easySPIN STEP_MODE register masks */
typedef enum {
	easySPIN_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
	easySPIN_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} easySPIN_STEP_MODE_Masks_TypeDef;

/* easySPIN STEP_MODE register options */
/* easySPIN STEP_SEL options */
typedef enum {
	easySPIN_STEP_SEL_1    = ((uint8_t) 0x08),
	easySPIN_STEP_SEL_1_2  = ((uint8_t) 0x09),
	easySPIN_STEP_SEL_1_4  = ((uint8_t) 0x0A),
	easySPIN_STEP_SEL_1_8  = ((uint8_t) 0x0B),
	easySPIN_STEP_SEL_1_16 = ((uint8_t) 0x0C)
} easySPIN_STEP_SEL_TypeDef;

/* easySPIN SYNC_SEL options */
typedef enum {
	easySPIN_SYNC_SEL_1_2    = ((uint8_t) 0x80),
	easySPIN_SYNC_SEL_1      = ((uint8_t) 0x90),
	easySPIN_SYNC_SEL_2      = ((uint8_t) 0xA0),
	easySPIN_SYNC_SEL_4      = ((uint8_t) 0xB0),
	easySPIN_SYNC_SEL_8      = ((uint8_t) 0xC0),
	easySPIN_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} easySPIN_SYNC_SEL_TypeDef;

/* easySPIN ALARM_EN register options */
typedef enum {
	easySPIN_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
	easySPIN_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
	easySPIN_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
	easySPIN_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
	easySPIN_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
	easySPIN_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} easySPIN_ALARM_EN_TypeDef;

/* easySPIN Config register masks */
typedef enum {
	easySPIN_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
	easySPIN_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
	easySPIN_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
	easySPIN_CONFIG_OC_SD    = ((uint16_t) 0x0080),
	easySPIN_CONFIG_POW_SR   = ((uint16_t) 0x0300),
	easySPIN_CONFIG_TSW      = ((uint16_t) 0x7C00)
} easySPIN_CONFIG_Masks_TypeDef;

/* easySPIN Config register options */
typedef enum {
	easySPIN_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
	easySPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
	easySPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
	easySPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
	easySPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
	easySPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
	easySPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
	easySPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
	easySPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
	easySPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
	easySPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
	easySPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
	easySPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} easySPIN_CONFIG_OSC_MGMT_TypeDef;

typedef enum {
	easySPIN_CONFIG_EN_TQREG_INT_REG = ((uint16_t) 0x0000),
	easySPIN_CONFIG_EN_TQREG_ADC_IN  = ((uint16_t) 0x0020)
} easySPIN_CONFIG_EN_TQREG_TypeDef;

typedef enum {
	easySPIN_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
	easySPIN_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} easySPIN_CONFIG_OC_SD_TypeDef;

typedef enum {
	easySPIN_CONFIG_SR_180V_us = ((uint16_t) 0x0000),
	easySPIN_CONFIG_SR_290V_us = ((uint16_t) 0x0200),
	easySPIN_CONFIG_SR_530V_us = ((uint16_t) 0x0300)
} easySPIN_CONFIG_POW_SR_TypeDef;

typedef enum {
	easySPIN_CONFIG_TSW_4_us   = (((uint16_t) 0x01) << 10),
	easySPIN_CONFIG_TSW_8_us   = (((uint16_t) 0x02) << 10),
	easySPIN_CONFIG_TSW_12_us  = (((uint16_t) 0x03) << 10),
	easySPIN_CONFIG_TSW_16_us  = (((uint16_t) 0x04) << 10),
	easySPIN_CONFIG_TSW_20_us  = (((uint16_t) 0x05) << 10),
	easySPIN_CONFIG_TSW_24_us  = (((uint16_t) 0x06) << 10),
	easySPIN_CONFIG_TSW_28_us  = (((uint16_t) 0x07) << 10),
	easySPIN_CONFIG_TSW_32_us  = (((uint16_t) 0x08) << 10),
	easySPIN_CONFIG_TSW_36_us  = (((uint16_t) 0x09) << 10),
	easySPIN_CONFIG_TSW_40_us  = (((uint16_t) 0x0A) << 10),
	easySPIN_CONFIG_TSW_44_us  = (((uint16_t) 0x0B) << 10),
	easySPIN_CONFIG_TSW_48_us  = (((uint16_t) 0x0C) << 10),
	easySPIN_CONFIG_TSW_52_us  = (((uint16_t) 0x0D) << 10),
	easySPIN_CONFIG_TSW_56_us  = (((uint16_t) 0x0E) << 10),
	easySPIN_CONFIG_TSW_60_us  = (((uint16_t) 0x0F) << 10),
	easySPIN_CONFIG_TSW_64_us  = (((uint16_t) 0x10) << 10),
	easySPIN_CONFIG_TSW_68_us  = (((uint16_t) 0x11) << 10),
	easySPIN_CONFIG_TSW_72_us  = (((uint16_t) 0x12) << 10),
	easySPIN_CONFIG_TSW_76_us  = (((uint16_t) 0x13) << 10),
	easySPIN_CONFIG_TSW_80_us  = (((uint16_t) 0x14) << 10),
	easySPIN_CONFIG_TSW_84_us  = (((uint16_t) 0x15) << 10),
	easySPIN_CONFIG_TSW_88_us  = (((uint16_t) 0x16) << 10),
	easySPIN_CONFIG_TSW_92_us  = (((uint16_t) 0x17) << 10),
	easySPIN_CONFIG_TSW_96_us  = (((uint16_t) 0x18) << 10),
	easySPIN_CONFIG_TSW_100_us = (((uint16_t) 0x19) << 10),
	easySPIN_CONFIG_TSW_104_us = (((uint16_t) 0x1A) << 10),
	easySPIN_CONFIG_TSW_108_us = (((uint16_t) 0x1B) << 10),
	easySPIN_CONFIG_TSW_112_us = (((uint16_t) 0x1C) << 10),
	easySPIN_CONFIG_TSW_116_us = (((uint16_t) 0x1D) << 10),
	easySPIN_CONFIG_TSW_120_us = (((uint16_t) 0x1E) << 10),
	easySPIN_CONFIG_TSW_124_us = (((uint16_t) 0x1F) << 10)
} easySPIN_CONFIG_TSW_TypeDef;

/* Status Register bit masks */
typedef enum {
	easySPIN_STATUS_HIZ         = (((uint16_t) 0x0001)),
	easySPIN_STATUS_DIR         = (((uint16_t) 0x0010)),
	easySPIN_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
	easySPIN_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
	easySPIN_STATUS_UVLO        = (((uint16_t) 0x0200)),
	easySPIN_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
	easySPIN_STATUS_TH_SD       = (((uint16_t) 0x0800)),
	easySPIN_STATUS_OCD         = (((uint16_t) 0x1000))
} easySPIN_STATUS_Masks_TypeDef;

/* Status Register options */
typedef enum {
	easySPIN_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
	easySPIN_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} easySPIN_STATUS_DIR_TypeDef;

/* easySPIN internal register addresses */
typedef enum {
	easySPIN_ABS_POS        = ((uint8_t) 0x01),
	easySPIN_EL_POS         = ((uint8_t) 0x02),
	easySPIN_MARK           = ((uint8_t) 0x03),
	easySPIN_RESERVED_REG01 = ((uint8_t) 0x04),
	easySPIN_RESERVED_REG02 = ((uint8_t) 0x05),
	easySPIN_RESERVED_REG03 = ((uint8_t) 0x06),
	easySPIN_RESERVED_REG04 = ((uint8_t) 0x07),
	easySPIN_RESERVED_REG05 = ((uint8_t) 0x08),
	easySPIN_RESERVED_REG06 = ((uint8_t) 0x15),
	easySPIN_TVAL           = ((uint8_t) 0x09),
	easySPIN_RESERVED_REG07 = ((uint8_t) 0x0A),
	easySPIN_RESERVED_REG08 = ((uint8_t) 0x0B),
	easySPIN_RESERVED_REG09 = ((uint8_t) 0x0C),
	easySPIN_RESERVED_REG10 = ((uint8_t) 0x0D),
	easySPIN_T_FAST         = ((uint8_t) 0x0E),
	easySPIN_TON_MIN        = ((uint8_t) 0x0F),
	easySPIN_TOFF_MIN       = ((uint8_t) 0x10),
	easySPIN_RESERVED_REG11 = ((uint8_t) 0x11),
	easySPIN_ADC_OUT        = ((uint8_t) 0x12),
	easySPIN_OCD_TH         = ((uint8_t) 0x13),
	easySPIN_RESERVED_REG12 = ((uint8_t) 0x14),
	easySPIN_STEP_MODE      = ((uint8_t) 0x16),
	easySPIN_ALARM_EN       = ((uint8_t) 0x17),
	easySPIN_CONFIG         = ((uint8_t) 0x18),
	easySPIN_STATUS         = ((uint8_t) 0x19),
	easySPIN_RESERVED_REG13 = ((uint8_t) 0x1A),
	easySPIN_RESERVED_REG14 = ((uint8_t) 0x1B)
} easySPIN_Registers_TypeDef;

/* easySPIN command set */
typedef enum {
	easySPIN_NOP           = ((uint8_t) 0x00),
	easySPIN_SET_PARAM     = ((uint8_t) 0x00),
	easySPIN_GET_PARAM     = ((uint8_t) 0x20),
	easySPIN_ENABLE        = ((uint8_t) 0xB8),
	easySPIN_DISABLE       = ((uint8_t) 0xA8),
	easySPIN_GET_STATUS    = ((uint8_t) 0xD0),
	easySPIN_RESERVED_CMD1 = ((uint8_t) 0xEB),
	easySPIN_RESERVED_CMD2 = ((uint8_t) 0xF8)
} easySPIN_Commands_TypeDef;

/* easySPIN movement direction options */
typedef enum {
	DIR_Forward = ((uint8_t) 0x01), DIR_Reverse = ((uint8_t) 0x00)
} easySPIN_Direction_TypeDef;

/* easySPIN action options */
typedef enum {
	ACTION_RESET = ((uint8_t) 0x00), ACTION_COPY = ((uint8_t) 0x01)
} easySPIN_Action_TypeDef;
/**
 * @}
 */

typedef struct {
	char device[20];
	uint32_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;
	int spi_fd;
} spidev;

typedef struct {
	spidev	spi_device;
	easySPIN_RegsStruct_TypeDef		regs;
	bool enabled;
} easySPIN_stepper;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void easySPIN_Peripherals_Init(void);
void easySPIN_Registers_Set(spidev *spi_dev, easySPIN_RegsStruct_TypeDef* easySPIN_RegsStruct);

/* Application Commands implementation ------------------------------------ */
void easySPIN_Nop(spidev *spi_dev);
void easySPIN_SetParam(spidev *spi_dev, easySPIN_Registers_TypeDef param, uint32_t value);
uint32_t easySPIN_GetParam(easySPIN_Registers_TypeDef param);
void easySPIN_Enable(spidev *spi_dev);
void easySPIN_Disable(spidev *spi_dev);
uint16_t easySPIN_Get_Status(spidev *spi_dev);

/* Additional Commands implementation ------------------------------------ */
void easySPIN_Reset(int gpio);
void easySPIN_ReleaseReset(int gpio);
//void easySPIN_DirectionSetup(easySPIN_Direction_TypeDef direction);

#endif /* __EASYSPIN_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
