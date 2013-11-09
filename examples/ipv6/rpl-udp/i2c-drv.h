#include <avr/io.h>

#ifndef I2CDRV_H_
#define I2CDRV_H_


#define I2C_HIGH_SPEED   0

#define I2C_WRITE    	 0x00
#define I2C_READ    	 0x01
#define I2C_START        0x08
#define I2C_REP_START    0x10
#define I2C_MT_SLA_ACK   0x18
#define I2C_MT_DATA_ACK  0x28
#define I2C_MR_SLA_ACK   0x40
#define I2C_MR_DATA_ACK  0x50
#define I2C_MR_DATA_NACK 0x58


void i2c_init(void);
void i2c_stop(void);

int8_t i2c_start(uint8_t addr);
int8_t i2c_rep_start(uint8_t addr);
int8_t i2c_write(uint8_t data);
int8_t i2c_read(uint8_t *data, uint8_t ack);
int8_t i2c_read_ack(uint8_t *data);
int8_t i2c_read_nack(uint8_t *data);

#endif