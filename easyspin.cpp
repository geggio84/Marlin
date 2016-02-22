/**
 ******************************************************************************
 * @file    easyspin.c
 * @author  EMEA AMS-IPD Marketing & Application, Prague - VE
 * @version V1.0.1
 * @date    June-2012
 * @brief   easySPIN (L6474) product related routines
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

/* Includes ------------------------------------------------------------------*/
#include "easyspin.h"
#include "gpio_lib.h"
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
uint8_t easySPIN_Write_Byte(uint8_t byte);

/* Private functions ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void pabort(const char *s)
{
	perror(s);
	abort();
}

/**
 * @brief  Transmits/Receives one byte to/from easySPIN over SPI.
 * @param  Transmited byte
 * @retval Received byte
 */
uint8_t easySPIN_Write_Byte(uint8_t byte) {

	int ret;
	uint8_t rx;

	struct spi_ioc_transfer tr = {
		tx_buf : (unsigned long)&byte,
		rx_buf : (unsigned long)&rx,
		len : sizeof(byte),
		speed_hz : speed,
		delay_usecs : delay,
		bits_per_word : bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	return (uint8_t) (rx);
}

/**
 * @brief  Initializes easySPIN related peripherals, disables powerstage and releases reset pin
 * @param  None
 * @retval None
 */
void easySPIN_Init(void) {

	/* Standby-reset deactivation */
	easySPIN_Reset(easySPIN_STBY_RESET_GPIO);
	sleep(1);

	/* Disable easySPIN powerstage */
	easySPIN_Disable();

	/* Standby-reset deactivation */
	easySPIN_ReleaseReset(easySPIN_STBY_RESET_GPIO);
}

/**
 * @brief  Fills-in easySPIN configuration structure with default values.
 * @param  Structure address (pointer to struct)
 * @retval None
 */
void easySPIN_Regs_Struct_Reset(
		easySPIN_RegsStruct_TypeDef* easySPIN_RegsStruct) {
	easySPIN_RegsStruct->ABS_POS = 0x00;
	easySPIN_RegsStruct->EL_POS = 0x00;
	easySPIN_RegsStruct->MARK = 0x00;

	easySPIN_RegsStruct->TVAL = 0x00;
	easySPIN_RegsStruct->T_FAST = easySPIN_TOFF_FAST_8_0_us
			| easySPIN_FAST_STEP_8_0_us;
	easySPIN_RegsStruct->TON_MIN = 0x00;
	easySPIN_RegsStruct->TOFF_MIN = 0x01;

	/* OCD_TH register setup */
	easySPIN_RegsStruct->OCD_TH = easySPIN_OCD_TH_2625mA;

	/* STEP_MODE register */
	easySPIN_RegsStruct->STEP_MODE = easySPIN_STEP_SEL_1
			| easySPIN_SYNC_SEL_1_2;

	/* ALARM_EN register setup */
	easySPIN_RegsStruct->ALARM_EN = easySPIN_ALARM_EN_OVERCURRENT
			| easySPIN_ALARM_EN_THERMAL_SHUTDOWN
			| easySPIN_ALARM_EN_THERMAL_WARNING
			| easySPIN_ALARM_EN_UNDERVOLTAGE | easySPIN_ALARM_EN_SW_TURN_ON
			| easySPIN_ALARM_EN_WRONG_NPERF_CMD;

	/* CONFIG register setup */
	easySPIN_RegsStruct->CONFIG = easySPIN_CONFIG_INT_16MHZ
			| easySPIN_CONFIG_EN_TQREG_INT_REG | easySPIN_CONFIG_OC_SD_ENABLE
			| easySPIN_CONFIG_SR_180V_us | easySPIN_CONFIG_TSW_8_us;
}

/**
 * @brief  Configures easySPIN internal registers with values in the config structure.
 * @param  Configuration structure address (pointer to configuration structure)
 * @retval None
 */
void easySPIN_Registers_Set(easySPIN_RegsStruct_TypeDef* easySPIN_RegsStruct) {
	easySPIN_SetParam(easySPIN_ABS_POS, easySPIN_RegsStruct->ABS_POS);
	easySPIN_SetParam(easySPIN_EL_POS, easySPIN_RegsStruct->EL_POS);
	easySPIN_SetParam(easySPIN_MARK, easySPIN_RegsStruct->MARK);
	easySPIN_SetParam(easySPIN_TVAL, easySPIN_RegsStruct->TVAL);
	easySPIN_SetParam(easySPIN_T_FAST, easySPIN_RegsStruct->T_FAST);
	easySPIN_SetParam(easySPIN_TON_MIN, easySPIN_RegsStruct->TON_MIN);
	easySPIN_SetParam(easySPIN_TOFF_MIN, easySPIN_RegsStruct->TOFF_MIN);
	easySPIN_SetParam(easySPIN_OCD_TH, easySPIN_RegsStruct->OCD_TH);
	easySPIN_SetParam(easySPIN_STEP_MODE, easySPIN_RegsStruct->STEP_MODE);
	easySPIN_SetParam(easySPIN_ALARM_EN, easySPIN_RegsStruct->ALARM_EN);
	easySPIN_SetParam(easySPIN_CONFIG, easySPIN_RegsStruct->CONFIG);
}

/* Application Commands implementation ----------------------------------------*/

/**
 * @brief  Issues easySPIN NOP command.
 * @param  None
 * @retval None
 */
void easySPIN_Nop(void) {
	/* Send NOP operation code to easySPIN */
	easySPIN_Write_Byte(easySPIN_NOP);
}

/**
 * @brief  Issues easySPIN SetParam command.
 * @param  easySPIN register address, value to be set
 * @retval None
 */
void easySPIN_SetParam(easySPIN_Registers_TypeDef param, uint32_t value) {

	printf("####### easySPIN_SetParam nr. 0x%X with value 0x%X #######\n",param,value);
	/* Send SetParam operation code to easySPIN */
	easySPIN_Write_Byte(easySPIN_SET_PARAM | param);
	switch (param) {
	case easySPIN_ABS_POS:
		;
	case easySPIN_MARK:
		/* Send parameter - byte 2 to easySPIN */
		easySPIN_Write_Byte((uint8_t) (value >> 16));
	case easySPIN_EL_POS:
		;
	case easySPIN_CONFIG:
		;
	case easySPIN_STATUS:
		/* Send parameter - byte 1 to easySPIN */
		easySPIN_Write_Byte((uint8_t) (value >> 8));
	default:
		/* Send parameter - byte 0 to easySPIN */
		easySPIN_Write_Byte((uint8_t) (value));
	}
}

/**
 * @brief  Issues easySPIN GetParam command.
 * @param  easySPIN register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 */
uint32_t easySPIN_GetParam(easySPIN_Registers_TypeDef param) {
	uint32_t temp = 0;
	uint32_t rx = 0;

	/* Send GetParam operation code to easySPIN */
	temp = easySPIN_Write_Byte(easySPIN_GET_PARAM | param);
	/* MSB which should be 0 */
	temp = temp << 24;
	rx |= temp;
	switch (param) {
	case easySPIN_ABS_POS:
		;
	case easySPIN_MARK:
		temp = easySPIN_Write_Byte((uint8_t) (0x00));
		temp = temp << 16;
		rx |= temp;
	case easySPIN_EL_POS:
		;
	case easySPIN_CONFIG:
		;
	case easySPIN_STATUS:
		temp = easySPIN_Write_Byte((uint8_t) (0x00));
		temp = temp << 8;
		rx |= temp;
	default:
		temp = easySPIN_Write_Byte((uint8_t) (0x00));
		rx |= temp;
	}
	return rx;
}

/**
 * @brief Issues easySPIN Enable command.
 * @param  None
 * @retval None
 */
void easySPIN_Enable(void) {
	/* Send Enable operation code to easySPIN */
	easySPIN_Write_Byte(easySPIN_ENABLE);
}

/**
 * @brief Issues easySPIN Disable command.
 * @param  None
 * @retval None
 */
void easySPIN_Disable(void) {
	/* Send Disable operation code to easySPIN */
	easySPIN_Write_Byte(easySPIN_DISABLE);
}

/**
 * @brief  Issues easySPIN GetStatus command.
 * @param  None
 * @retval Status Register content
 */
uint16_t easySPIN_Get_Status(void) {
	uint16_t temp = 0;
	uint16_t rx = 0;

	/* Send GetStatus operation code to easySPIN */
	easySPIN_Write_Byte(easySPIN_GET_STATUS);
	/* Send zero byte / receive MSByte from easySPIN */
	temp = easySPIN_Write_Byte((uint8_t) (0x00));
	temp <<= 8;
	rx |= temp;
	/* Send zero byte / receive LSByte from easySPIN */
	temp = easySPIN_Write_Byte((uint8_t) (0x00));
	rx |= temp;
	printf("easySPIN_Get_Status = 0x%X\n",rx);
	return rx;
}

/**
 * @brief  Checks easySPIN Flag signal.
 * @param  None
 * @retval one if Flag signal is active, otherwise zero
 */
uint8_t easySPIN_Flag(void) {
	if (!(get_gpio_value(easySPIN_FLAG_GPIO)))
		return 0x01;
	else
		return 0x00;
}

/* Additional Application Commands implementation -----------------------------*/

/**
 *	@brief  Resets easySPIN by STBY/RESET signal activation
 * @param  None
 *	@retval None
 */
void easySPIN_Reset(int gpio) {
	/* Standby-reset signal activation - low */
	set_gpio_value(gpio, LOW);
}

/**
 * @brief  Release STBY/RESET signal on easySPIN
 *	@param  None
 * @retval None
 */
void easySPIN_ReleaseReset(int gpio) {
	/* Standby-reset signal de-activation - high */
	set_gpio_value(gpio, HIGH);
}

/**
 * @brief	easySPIN movement direction setup
 * @param	direction: specifies the direction of movement
 *   This parameter can be either DIR_Forward or DIR_Reverse
 * @retval None
 */
/*void easySPIN_DirectionSetup(easySPIN_Direction_TypeDef direction) {
	if (direction == DIR_Forward) {
		set_gpio_value(easySPIN_DIR_GPIO, HIGH);
	} else {
		set_gpio_value(easySPIN_DIR_GPIO, LOW);
	}
}*/
