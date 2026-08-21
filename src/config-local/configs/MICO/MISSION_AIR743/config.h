/*
 * Mission-computer configuration for MicoAir743 hardware.
 *
 * This file is part of Betaflight and is distributed under the terms of the
 * GNU General Public License, version 3 or (at your option) any later version.
 */

#pragma once

#define FC_TARGET_MCU   STM32H743

#define BOARD_NAME      MISSION_AIR743
#define MANUFACTURER_ID MICO

/*
 * Select features explicitly.  CLOUD_BUILD prevents common_pre.h from adding
 * the normal flight-controller/FPV feature set (physical/serial RX,
 * telemetry, OSD, VTX, servos, Blackbox and other pilot-facing modules).
 */
#define CLOUD_BUILD
#define MISSION_COMPUTER_BUILD
#define DEFAULT_RX_FEATURE 0

/* Keep only the RX parameter group required by the legacy Betaflight core.
 * No physical or serial RC receiver protocol is enabled by this target. */
#define USE_RX_MSP

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_DPS310
#define USE_MAG
#define USE_MAG_IST8310
#define USE_GPS
#define USE_SDCARD

/* Compatibility gates required by the current Betaflight scheduler, MSP and
 * GPS code.  No motor pins or ESC protocol are exposed, and GPS Rescue is not
 * selected as a runtime failsafe, so the target has zero physical outputs. */
#define USE_MOTOR
#define USE_GPS_RESCUE
#define USE_SIMPLIFIED_TUNING

/* General-purpose serial ports.  None is assigned to RX, ESC or DisplayPort. */
#define UART1_TX_PIN PA9
#define UART2_TX_PIN PA2
#define UART3_TX_PIN PD8
#define UART4_TX_PIN PA0
#define UART6_TX_PIN PC6
#define UART7_TX_PIN PE8
#define UART8_TX_PIN PE1
#define UART1_RX_PIN PA10
#define UART2_RX_PIN PA3
#define UART3_RX_PIN PD9
#define UART4_RX_PIN PA1
#define UART6_RX_PIN PC7
#define UART7_RX_PIN PE7
#define UART8_RX_PIN PE0

#define I2C1_SCL_PIN PB6
#define I2C2_SCL_PIN PB10
#define I2C1_SDA_PIN PB7
#define I2C2_SDA_PIN PB11

/* The tested board has BMI270 on SPI2, as in MICOAIR743 (not V2/SPI3). */
#define SPI2_SCK_PIN PD3
#define SPI2_SDI_PIN PC2
#define SPI2_SDO_PIN PC3
#define GYRO_1_CS_PIN PA15
#define GYRO_1_SPI_INSTANCE SPI2

#define SDIO_CK_PIN  PC12
#define SDIO_CMD_PIN PD2
#define SDIO_D0_PIN  PC8
#define SDIO_D1_PIN  PC9
#define SDIO_D2_PIN  PC10
#define SDIO_D3_PIN  PC11
#define SDIO_USE_4BIT 1
#define SDIO_DEVICE   SDIODEV_1

#define ADC_VBAT_PIN PC0
#define ADC_CURR_PIN PC1
#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

#define LED0_PIN PE4
#define LED1_PIN PE6
#define LED2_PIN PE5

#define BARO_I2C_INSTANCE I2CDEV_2
#define MAG_I2C_INSTANCE  I2CDEV_2
#define MAG_I2C_ADDRESS   14
#define MAG_ALIGN         CW90_DEG
#define MAG_ALIGN_YAW     900

#define MSP_UART SERIAL_PORT_USART1
#define GPS_UART SERIAL_PORT_USART3

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE  402
#define DEFAULT_VOLTAGE_METER_SCALE  213
