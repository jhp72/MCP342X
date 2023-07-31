#ifdef MCP3428
/*
 * mcp3428.h
 *
 *  Created on: 2023.04.01.
 *      Author: jaehong park (cto.osfactory@gmail.com)
 */

#ifndef __MCP3428_H__
#define __MCP3428_H__

#include "nx_config.h"

// I2C Address of device
// MCP3421, MCP3425 & MCP3426 are factory programed for any of 0x68 thru 0x6F
//#define MCP342X_DEFAULT_ADDRESS	0x68

// MCP3422, MCP3423, MCP3424, MCP3427 & MCP3428 addresses are controlled by address lines A0 and A1
// each address line can be low (GND), high (VCC) or floating (FLT)
#define MCP342X_A0GND_A1GND		0x68
#define MCP342X_A0GND_A1FLT		0x69
#define MCP342X_A0GND_A1VCC		0x6A
#define MCP342X_A0FLT_A1GND		0x6B
#define MCP342X_A0VCC_A1GND		0x6C
#define MCP342X_A0VCC_A1FLT		0x6D
#define MCP342X_A0VCC_A1VCC		0x6E
#define MCP342X_A0FLT_A1VCC		0x6F


// Conversion mode definitions
#define MCP342X_MODE_ONESHOT	0x00
#define MCP342X_MODE_CONTINUOUS	0x10


// Channel definitions
// MCP3421 & MCP3425 have only the one channel and ignore this param
// MCP3422, MCP3423, MCP3426 & MCP3427 have two channels and treat 3 & 4 as repeats of 1 & 2 respectively
// MCP3424 & MCP3428 have all four channels
#define	MCP342X_CHANNEL_1	0x00
#define	MCP342X_CHANNEL_2	0x20
#define	MCP342X_CHANNEL_3	0x40
#define	MCP342X_CHANNEL_4	0x60
#define	MCP342X_CHANNEL_MASK	0x60


// Sample size definitions - these also affect the sampling rate
// 12-bit has a max sample rate of 240sps
// 14-bit has a max sample rate of  60sps
// 16-bit has a max sample rate of  15sps
// 18-bit has a max sample rate of   3.75sps (MCP3421, MCP3422, MCP3423, MCP3424 only)
#define MCP342X_SIZE_12BIT	0x00
#define MCP342X_SIZE_14BIT	0x04
#define MCP342X_SIZE_16BIT	0x08
#define MCP342X_SIZE_18BIT	0x0C
#define MCP342X_SIZE_MASK	0x0C


// Programmable Gain definitions
#define MCP342X_GAIN_1X	0x00
#define MCP342X_GAIN_2X	0x01
#define MCP342X_GAIN_4X	0x02
#define MCP342X_GAIN_8X	0x03
#define MCP342X_GAIN_MASK 0x03


// /RDY bit definition
#define MCP342X_RDY	0x80
#define MCP342X_START      0X80 // write: start a conversion
#define MCP342X_BUSY       0X80 // read: output not ready


#define USE_ADC_2CH

void mcp3428_init(void);
void mcp3428_process(void);
void cli_dev_mcp3428_rst(void);


#endif //__MCP3428_H__


#endif//#ifdef MCP3428
