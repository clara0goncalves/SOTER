#ifndef MMA8452Q_H_
#define MMA8452Q_H_

#include <stdint.h>

#define MMA8452Q_STATUS			0X00
#define MMA8452Q_OUT_X_MSB		0x01
#define MMA8452Q_OUT_X_LSB		0x02
#define MMA8452Q_OUT_Y_MSB		0x03
#define MMA8452Q_OUT_Y_LSB		0x04
#define MMA8452Q_OUT_Z_MSB		0x05
#define MMA8452Q_OUT_Z_LSB		0x06
#define MMA8452Q_SYSMOD			0x0B
#define MMA8452Q_INT_SOURCE 	0x0C
#define MMA8452Q_WHO_AM_I		0x0D
#define MMA8452Q_XYZ_DATA_CFG	0x0E
#define MMA8452Q_CTRL_REG1		0x2A
#define MMA8452Q_CTRL_REG2		0x2B
#define MMA8452Q_CTRL_REG3		0x2C
#define MMA8452Q_CTRL_REG4		0x2D
#define MMA8452Q_CTRL_REG5		0x2E
#define MMA8452Q_ADDR			0x3A

extern void I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data);
extern uint8_t I2C_read(uint8_t dev_addr, uint8_t reg_addr);

extern void delay_ms(int time);

#endif /* MMA8452Q_H_ */
