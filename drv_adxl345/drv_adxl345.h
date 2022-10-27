/*
 * drv_adxl345.h
 *
 *  Created on: 24 oct. 2022
 *      Author: laurentf
 */

#ifndef DRV_ADXL345_H_
#define DRV_ADXL345_H_

#include <stdint.h>

typedef enum adxl345_command_enum
{
	CMD_READ       = 0x80,
	CMD_WRITE      = 0x00,
	CMD_MULTIBYTE  = 0x40,
	CMD_SINGLEBYTE = 0x00
} adxl345_command_t;

typedef enum adxl345_address_enum
{
	ADDR_DEVID       = 0x00,
	ADDR_BW_RATE     = 0x2C,
	ADDR_POWER_CTL   = 0x2D,
	ADDR_INT_ENABLE  = 0x2E,
	ADDR_DATA_FORMAT = 0x31,
	ADDR_DATA_X0     = 0x32
} adxl345_address_t;

//valeurs de rate trouvées dans la datasheet
typedef enum adxl345_rate_enum
{
	RATE_3200_HZ = 0xF,		// 1111
	RATE_1600_HZ = 0xE,		// 1110
	RATE_800_HZ  = 0xD,		// 1101
	RATE_400_HZ  = 0xC,		// 1100
	RATE_200_HZ  = 0xB,		// 1011
	RATE_100_HZ  = 0xA,		// 1010
	RATE_50_HZ   = 0x9,		// 1001
	RATE_25_HZ   = 0x8,		// 1000
	RATE_12_5_HZ = 0x7,		// 0111
	RATE_6_25_HZ = 0x6,		// 0110
	RATE_3_13_HZ = 0x5,		// 0101
	RATE_1_56_HZ = 0x4,		// 0100
	RATE_0_78_HZ = 0x3,		// 0011
	RATE_0_39_HZ = 0x2,		// 0010
	RATE_0_20_HZ = 0x1,		// 0001
	RATE_0_10_HZ = 0x0		// 0000
} adxl345_rate_t;

//valeurs de range trouvées sur datasheet
typedef enum adxl345_range_enum
{
	RANGE_2G  = 0x0,
	RANGE_4G  = 0x1,
	RANGE_8G  = 0x2,
	RANGE_16G = 0x3
} adxl345_range_t;

//pointeurs de fonctions ici
//adxl345_transmit_drv_t a chaque fois qu'elle est appelée on crée une structure de fonction
typedef int (* adxl345_transmit_drv_t)(uint8_t address, uint8_t *p_data, uint16_t size);
typedef int (* adxl345_receive_drv_t)(uint8_t address, uint8_t *p_data, uint16_t size);

typedef struct adxl345_serial_drv_struct
{
	adxl345_transmit_drv_t transmit;
	adxl345_receive_drv_t receive;
} adxl345_serial_drv_t;

typedef struct h_adxl345_struct
{
    // Rate structure crée précéemment mais on y met un élèment
	adxl345_rate_t adxl345_rate;
    // Range
	adxl345_range_t adxl345_range;

	// driver SPI
	adxl345_serial_drv_t adxl345_serial_drv;

    // x, y, z
	float x;
	float y;
	float z;

    // Mesure dispo
	uint8_t data_available;

    // Skipped data
	uint32_t skipped_data;

    // DEVID
	uint8_t devid;

} h_adxl345_t;

int adxl345_init(h_adxl345_t * h_adxl345);
int adxl345_start(h_adxl345_t * h_adxl345);

int adxl345_data_available(h_adxl345_t * h_adxl345);
int adxl345_read_acceleration(h_adxl345_t * h_adxl345);

float adxl345_get_x_acceleration(h_adxl345_t * h_adxl345);
float adxl345_get_y_acceleration(h_adxl345_t * h_adxl345);
float adxl345_get_z_acceleration(h_adxl345_t * h_adxl345);

void adxl345_irq_cb(h_adxl345_t * h_adxl345);

#endif /* DRV_ADXL345_H_ */
