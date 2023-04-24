/**
 * @file bmi_controller.h
 * @author Phil (zetsuboulevel@gmail.com)
 * @brief Controller for BMI160 6DOF MMU sensor.
 * @version 0.1
 * @date 2023-04-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BMI_CONTROLLER_H
#define BMI_CONTROLLER_H

#include "defines.h"
#include "nrf_drv_twi.h"

#define BMI_ADDR	(0x69<<0)

#define BMI_WHO_AM_I_REG		0x00
#define BMI_ERR_REG				0x02
#define BMI_PMU_STATUS_REG		0x03
#define BMI_DATA0_REG			0x04
#define BMI_DATA1_REG			0x05
#define BMI_DATA2_REG			0x06
#define BMI_DATA3_REG			0x07
#define BMI_DATA4_REG			0x08
#define BMI_DATA5_REG			0x09
#define BMI_DATA6_REG			0x0A
#define BMI_DATA7_REG			0x0B
#define BMI_DATA8_REG			0x0C
#define BMI_DATA9_REG			0x0D
#define BMI_DATA10_REG			0x0E
#define BMI_DATA11_REG			0x0F
#define BMI_DATA12_REG			0x10
#define BMI_DATA13_REG			0x11
#define BMI_DATA14_REG			0x12
#define BMI_DATA15_REG			0x13
#define BMI_DATA16_REG			0x14
#define BMI_DATA17_REG			0x15
#define BMI_DATA18_REG			0x16
#define BMI_DATA19_REG			0x17
#define BMI_SENSORTIME0_REG		0x18
#define BMI_SENSORTIME1_REG		0x19
#define BMI_SENSORTIME2_REG		0x1A
#define BMI_STATUS_REG			0x1B
#define BMI_INT_STATUS0_REG		0x1C
#define BMI_INT_STATUS1_REG		0x1D
#define BMI_INT_STATUS2_REG		0x1E
#define BMI_INT_STATUS3_REG		0x1F
#define BMI_TEMP0_REG			0x20
#define BMI_TEMP1_REG			0x21
#define BMI_FIFO_LENGTH0_REG	0x22
#define BMI_FIFO_LENGTH1_REG	0x23
#define BMI_FIFO_DATA_REG		0x24
#define BMI_ACC_CONF_REG		0x40
#define BMI_ACC_RANGE_REG		0x41
#define BMI_GYRO_CONF_REG		0x42
#define BMI_GYRO_RANGE_REG		0x43
#define BMI_MAG_CONF_REG		0x44
#define BMI_FIFO_DOWNS_REG		0x45
#define BMI_FIFO_CONFIG0_REG	0x46
#define BMI_FIFO_CONFIG1_REG	0x47
#define BMI_MAG_IF0_REG			0x4B
#define BMI_MAG_IF1_REG			0x4C
#define BMI_MAG_IF2_REG			0x4D
#define BMI_MAG_IF3_REG			0x4E
#define BMI_MAG_IF4_REG			0x4F
#define BMI_INT_EN0_REG			0x50
#define BMI_INT_EN1_REG			0x51
#define BMI_INT_EN2_REG			0x52
#define BMI_INT_OUT_CTRL_REG	0x53
#define BMI_INT_LATCH_REG		0x54
#define BMI_INT_MAP0_REG		0x55
#define BMI_INT_MAP1_REG		0x56
#define BMI_INT_MAP2_REG		0x57
#define BMI_INT_DATA0_REG		0x58
#define BMI_INT_DATA1_REG		0x59
#define BMI_INT_LOWHIGH0_REG	0x5A
#define BMI_INT_LOWHIGH1_REG	0x5B
#define BMI_INT_LOWHIGH2_REG	0x5C
#define BMI_INT_LOWHIGH3_REG	0x5D
#define BMI_INT_LOWHIGH4_REG	0x5E
#define BMI_INT_MOTION0_REG		0x5F
#define BMI_INT_MOTION1_REG		0x60
#define BMI_INT_MOTION2_REG		0x61
#define BMI_INT_MOTION3_REG		0x62
#define BMI_INT_TAP0_REG		0x63
#define BMI_INT_TAP1_REG		0x64
#define BMI_ORIENT0_REG			0x65
#define BMI_ORIENT1_REG			0x66
#define BMI_INT_FLAT0_REG		0x67
#define BMI_INT_FLAT1_REG		0x68
#define BMI_FOC_CONF_REG		0x69
#define BMI_CONF_REG			0x6A
#define BMI_IF_CONF_REG			0x6B
#define BMI_PMU_TRIGGER_REG		0x6C
#define BMI_SELF_TEST_REG		0x6D
#define BMI_NV_CONF_REG			0x70
#define BMI_OFFSET0_REG			0x71
#define BMI_OFFSET1_REG			0x72
#define BMI_OFFSET2_REG			0x73
#define BMI_OFFSET3_REG			0x74
#define BMI_OFFSET4_REG			0x75
#define BMI_OFFSET5_REG			0x76
#define BMI_OFFSET6_REG			0x77
#define BMI_STEP_CNT0_REG		0x78
#define BMI_STEP_CNT1_REG		0x79
#define BMI_STEP_CONF0_REG		0x7A
#define BMI_STEP_CONF1_REG		0x7B
#define BMI_CMD_REG				0x7E

#define BMI_ID					0xD1




struct BMI_InitStruct
{
	uint8_t scl_pin;
	uint8_t sda_pin;
	uint8_t i2c_instance;
};

struct MMU_Data
{
	float x,y,z;
};


void BMI_Init(struct BMI_InitStruct* bmi);
bool BMI_Setup();
void BMI_ReadData();
void BMI_SetupIRQ();
void BMI_CalibrateGyro();
struct MMU_Data BMI_GetGyro();
struct MMU_Data BMI_GetAccel();

#endif
