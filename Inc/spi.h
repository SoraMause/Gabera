/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
// MPU6500ã?��?®ãƒ¬ã‚¸ã‚¹ã‚¿ãƒžãƒ�????¿½?¿½??¿½?¿½?
#define MPU6500_RA_SELF_TEST_X_GYRO	0x00
#define MPU6500_RA_SELF_TEST_Y_GYRO	0x01
#define MPU6500_RA_SELF_TEST_Z_GYRO	0x02
#define MPU6500_RA_SELF_TEST_X_ACCEL	0x0D
#define MPU6500_RA_SELF_TEST_Y_ACCEL	0x0E
#define MPU6500_RA_SELF_TEST_Z_ACCEL	0x0F
#define MPU6500_RA_XG_OFFSET_H		0x13
#define MPU6500_RA_XG_OFFSET_L		0x14
#define MPU6500_RA_YG_OFFSET_H		0x15
#define MPU6500_RA_YG_OFFSET_L		0x16
#define MPU6500_RA_ZG_OFFSET_H		0x17
#define MPU6500_RA_ZG_OFFSET_L		0x18
#define MPU6500_RA_SMPLRT_DIV		0x19
#define MPU6500_RA_CONFIG			0x1A
#define MPU6500_RA_GYRO_CONFIG		0x1B
#define MPU6500_RA_ACCEL_CONFIG		0x1C
#define MPU6500_RA_ACCEL_CONFIG2	0x1D
#define MPU6500_RA_LP_ACCEL_ODR		0x1E
#define MPU6500_RA_WOM_THR			0x1F
#define MPU6500_RA_FIFO_EN			0x23
// ...
// å¤–éƒ¨I2Cå‘¨ã‚�?ï¿½????¿½?¿½??¿½?¿½çœ�?ç•¥
// ...
#define MPU6500_RA_INT_PIN_CFG		0x37
#define MPU6500_RA_INT_ENABLE		0x38
#define MPU6500_RA_INT_STATUS		0x3A
#define MPU6500_RA_ACCEL_XOUT_H		0x3B
#define MPU6500_RA_ACCEL_XOUT_L		0x3C
#define MPU6500_RA_ACCEL_YOUT_H		0x3D
#define MPU6500_RA_ACCEL_YOUT_L		0x3E
#define MPU6500_RA_ACCEL_ZOUT_H		0x3F
#define MPU6500_RA_ACCEL_ZOUT_L		0x40
#define MPU6500_RA_TEMP_OUT_H		0x41
#define MPU6500_RA_TEMP_OUT_L		0x42
#define MPU6500_RA_GYRO_XOUT_H		0x43
#define MPU6500_RA_GYRO_XOUT_L		0x44
#define MPU6500_RA_GYRO_YOUT_H		0x45
#define MPU6500_RA_GYRO_YOUT_L		0x46
#define MPU6500_RA_GYRO_ZOUT_H		0x47
#define MPU6500_RA_GYRO_ZOUT_L		0x48
// ...
// å¤–éƒ¨I2Cå‘¨ã‚�?ï¿½????¿½?¿½??¿½?¿½çœ�?ç•¥
// ...
#define MPU6500_RA_SIGNAL_PATH_RESET	0x68
#define MPU6500_RA_MOT_DETECT_CTRL		0x69
#define MPU6500_RA_USER_CTRL		0x6A
#define MPU6500_RA_PWR_MGMT_1		0x6B
#define MPU6500_RA_PWR_MGMT_2		0x6C
#define MPU6500_RA_FIFO_COUNTH		0x72
#define MPU6500_RA_FIFO_COUNTL			0x73
#define MPU6500_RA_FIFO_R_W			0x74
#define MPU6500_RA_WHO_AM_I			0x75
#define MPU6500_RA_XA_OFFSET_H		0x77
#define MPU6500_RA_XA_OFFSET_L		0x78
#define MPU6500_RA_YA_OFFSET_H		0x7A
#define MPU6500_RA_YA_OFFSET_L		0x7B
#define MPU6500_RA_ZA_OFFSET_H		0x7D
#define MPU6500_RA_ZA_OFFSET_L		0x7E

#define MPU6500_DEVICE_ID			0x70
// ã‚¸ãƒ£ã‚¤ãƒ­ã?��?®ã‚²ã‚¤ãƒ³ã?��?Œãƒ�??��ãƒ«ã‚¹ã‚±ãƒ¼ãƒ«ã?��?§2000dpsã?��?®æ™‚ï¿½????¿½?¿½??¿½?¿½å€¤
#define GYRO_FACTOR  16.4f

#define ACCEL_FACTOR 2048.0f
/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t read_byte( uint8_t reg );
int16_t read_shift_byte( uint8_t reg ); 
void write_byte( uint8_t reg, uint8_t val );
void MPU6500_init( void );
float MPU6500_read_gyro_z( void );
void MPU6500_z_axis_offset_calc_start( void );
void MPU6500_z_axis_offset_calc( void );
int8_t MPU6500_calc_check( void );
float checkGyroOffset( void );
float MPU6500_read_accel_x( void );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
