#ifdef MCP3428
/*
 * mcp3428.c
 *
 *  Created on: 2023.04.01.
 *      Author: jaehong park (smilemacho@gmail.com)
 */

/*
MIT License

Copyright (c) [2023.04.01] [jaehong park]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include PLATFORM_HEADER
#include "stack/include/ember-types.h"
#include "stack/include/event.h"
#include "hal/hal.h"
#include "hal/plugin/i2c-driver/i2c-driver.h"
#include "hal/micro/micro.h"
#include "zigbee_common.h"
#include "i2cspm.h"
#include "mcp3428.h"
#include "app/framework/include/af.h"


#define MCP342X_DEFAULT_ADDRESS (0x68 << 1)
#define MCP342X_DEFAULT_TIMEOUT (100)//100ms
//#define LONG_MIN                (-1)

typedef enum {
    OneShot = 0,
    Continuous
} Conversion;

typedef enum {
    x1 = 0,
    x2,
    x4,
    x8
} PGA;

typedef enum {
    _12bit = 0,
    _14bit,
    _16bit,
    _18bit
} Resolution;

static float	stepSizeTbl[] = {
		0.001,		// 12-bit, 1X Gain
		0.0005,		// 12-bit, 2X Gain
		0.00025,	// 12-bit, 4X Gain
		0.000125,	// 12-bit, 8X Gain
		0.00025,	// 14-bit, 1X Gain
		0.000125,	// 14-bit, 2X Gain
		0.0000625,	// 14-bit, 4X Gain
		0.00003125,	// 14-bit, 8X Gain
		0.0000625,	// 16-bit, 1X Gain
		0.00003125,	// 16-bit, 2X Gain
		0.000015625,	// 16-bit, 4X Gain
		0.0000078125,	// 16-bit, 8X Gain
		0.000015625,	// 18-bit, 1X Gain
		0.0000078125,	// 18-bit, 2X Gain
		0.00000390625,	// 18-bit, 4X Gain
		0.000001953125	// 18-bit, 8X Gain
};

uint8_t _address = MCP342X_DEFAULT_ADDRESS;//slave_adr
uint32_t _freq = 400000;
char _config[4];//4ch
char _channel = 0;
char _Buffer[4];
int32u readWaitTime = 0;//millisecond

// I2C_TransferReturn_TypeDef
// Return codes for read/write transaction error handling
/*
#define I2C_DRIVER_ERR_NONE         0x00
#define I2C_DRIVER_ERR_TIMEOUT      0x01
#define I2C_DRIVER_ERR_ADDR_NAK     0x02
#define I2C_DRIVER_ERR_DATA_NAK     0x03
#define I2C_DRIVER_ERR_ARB_LOST     0x04
#define I2C_DRIVER_ERR_USAGE_FAULT  0x05
#define I2C_DRIVER_ERR_SW_FAULT     0x06
#define I2C_DRIVER_ERR_UNKOWN       0xFF
*/
bool MCP3428Status = false;


void MCP342X_i2cInit(void );//400000
bool MCP342X_init(void);
bool MCP342X_config(uint8_t channel, Resolution res, Conversion mode, PGA gain);
int32_t MCP342X_read(uint8_t channel);
int32_t MCP342X_readVoltage(uint8_t channel);
bool MCP342X_newConversion(uint8_t channel);
int32_t MCP342x_getResult(uint8_t channel);
int8_t MCP342X_isConversionFinished(uint8_t channel);

bool I2C_generalCallReset(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x06;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	//MCP3428Status = false;
		emberAfAppPrintln("Failed to I2C_generalCallReset...ack: %d", ack);
    } else {
    	//MCP3428Status = true;
		emberAfAppPrintln("Succeeded to I2C_generalCallReset...ack: %d", ack);
    }

    return (ack == 0);
}

bool I2C_generalCallLatch(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x04;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	//MCP3428Status = false;
		emberAfAppPrintln("Failed to I2C_generalCallLatch...ack: %d", ack);
    } else {
    	//MCP3428Status = true;
		emberAfAppPrintln("Succeeded to I2C_generalCallLatch...ack: %d", ack);
    }

    return (ack == 0);
}

bool I2C_generalCallConvert(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x08;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	//MCP3428Status = false;
		emberAfAppPrintln("Failed to I2C_generalCallConvert...ack: %d", ack);
    } else {
    	//MCP3428Status = true;
		emberAfAppPrintln("Succeeded to I2C_generalCallConvert...ack: %d", ack);
    }

    return (ack == 0);
}

void mcp342x_i2c_init(void)
{
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
  	I2CSPM_Init(&i2cInit);

	emberAfAppPrintln("Init I2CSPM_Init() ...");
}


void MCP342X_i2cInit(void) {
    // _address = slave_adr;
	// _freq = freq;
    mcp342x_i2c_init();
}


bool MCP342X_init(void) {
    int32_t ack = -1;

    _config[0] = 0x10;//0b00010000;  // channel 1, continuous mode, 12bit
    _config[1] = 0x30;//0b00110000;  // channel 2, continuous mode, 12bit
    _config[2] = 0x50;//0b01010000;  // channel 3, continuous mode, 12bit
    _config[3] = 0x70;//0b01110000;  // channel 4, continuous mode, 12bit

    memset(_Buffer, 0, sizeof(_Buffer));

    //_i2c->lock();
    ack = halI2cWriteBytes(_address, &_config[0], 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	MCP3428Status = false;
		emberAfAppPrintln("Failed to init mcp3428...ack: %d", ack);
    } else {
    	MCP3428Status = true;
		emberAfAppPrintln("Succeeded to init mcp3428...ack: %d", ack);
    }

    return (ack == 0);
}

bool MCP342X_config(uint8_t channel, Resolution res, Conversion mode, PGA gain) {
    int32_t ack = -1;
    _config[channel] |= ((res << 2) | gain);
    _config[channel] ^= (-mode ^ _config[channel]) & (1 << 4);

    //_i2c->lock();
    ack = halI2cWriteBytes(_address, &_config[channel], 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	MCP3428Status = false;
		emberAfAppPrintln("Failed to config mcp3428...ack: %d", ack);
    } else {
    	MCP3428Status = true;
		emberAfAppPrintln("Succeeded to config mcp3428...ack: %d", ack);
    }

    return (ack == 0);
}

bool MCP342X_newConversion(uint8_t channel) {
    char byte = _config[channel] |= 128;
    int32_t ack = -1;

    memset(_Buffer, 0, sizeof(_Buffer));

    //_i2c->lock();
    ack = halI2cWriteBytes(_address, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	MCP3428Status = false;
		emberAfAppPrintln("Failed to newConversion mcp3428...ack: %d", ack);
    } else {
    	MCP3428Status = true;
		emberAfAppPrintln("Succeeded to newConversion mcp3428...ack: %d", ack);
    }

    return (ack == 0);
}


/**
 * @brief Check in conversion is done
 * 
 * @param channel 
 * @return 0 if not done, 1 if done, -1 in case of error
 */
int8_t MCP342X_isConversionFinished(uint8_t channel) {
    char requested_bytes = 4;
    int32_t ack = -1;

    if (((_config[channel] >> 2) & 0x03) != 0x03) {  //0b11 // not 18bit
        requested_bytes = 3;
    }

    memset(_Buffer, 0, sizeof(_Buffer));

    //_i2c->lock();
    ack = halI2cReadBytes(_address, _Buffer, requested_bytes);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
    	MCP3428Status = false;
		emberAfAppPrintln("Failed to ConversionFinished mcp3428...ack: %d", ack);
    } else {
    	MCP3428Status = true;
		emberAfAppPrintln("Succeeded to ConversionFinished mcp3428...ack: %d", ack);
    }

    if (ack == 0 && (_Buffer[requested_bytes - 1] >> 7) != 0x00) {
        if (((_Buffer[requested_bytes - 1] >> 5) & 0x03) == channel) {//0b11
            //halI2cReadBytes(0);  // send NACK
            //halI2cReadBytes(_address, 0, 0);
            emberAfAppPrintln("ConversionFinished...return to TRUE...channel: %d, _Buffer[%d]: %d",channel, requested_bytes - 1,  _Buffer[requested_bytes - 1]);
            return 1;  // data ready
        }
    }

    // TODO -1
    return 0;  // not ready
}


int32_t MCP342X_getResult(uint8_t channel) {
    int32_t result = LONG_MIN;
    int16_t tmp = 0;
    uint8_t resolution = ((_config[channel] >> 2) & 0x03);//0b11

    switch (resolution) {
        case _12bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x0FFF;  // substract 12bit

            result = (int32_t)tmp;

            break;
        }

        case _14bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);
            tmp &= 0x3FFF;  // substract 14bit

            result = (int32_t)tmp;

            break;
        }

        case _16bit: {
            tmp = ((_Buffer[0] << 8) | _Buffer[1]);

            result = (int32_t)tmp;

            break;
        }

        case _18bit: {
            result = ((_Buffer[0] << 16) | (_Buffer[1] << 8) | _Buffer[2]);
            result &= 0x3FFFF;  // substract 18bit

            break;
        }
    }

#if (1)
    if(result > 2047){
			result -= 4095;
	}
#endif

    emberAfAppPrintln("MCP342X_getResult...channel: %d, result: %d", channel, result);
    return result;
}

uint32_t MCP342x_Resolution_getConversionDelayTime(uint8_t resolution){
  switch (resolution) {
  case 12:
    return 4167; // 240 SamplesPerSecond
  case 14:
    return 16667; // 60 SPS
  case 16:
    return 66667; // 15 SPS
  case 18:
    return 266667; // 3.75 SPS
  }
  return 0; // Shouldn't happen
}

int32_t MCP342X_read(uint8_t channel) {
    uint8_t resolution = ((_config[channel] >> 2) & 0x03);//0b11
    uint16_t delay = 4; //4ms

    if (((_config[channel] >> 4) & 1) == OneShot) {
        if (!MCP342X_newConversion(channel)) {
            emberAfAppPrintln("MCP342X_read...return LONG_MIN...OneShot, channel: %d", channel);
            return LONG_MIN;
        }
    }

	delay = (resolution == _12bit ? 4 : (resolution == _14bit ? 16 : (resolution == _16bit ? 66 : 266)));
	halCommonDelayMilliseconds(delay);

    readWaitTime = halCommonGetInt32uMillisecondTick();

    while (MCP342X_isConversionFinished(channel) == 1) {
        if (elapsedTimeInt32u(readWaitTime, halCommonGetInt32uMillisecondTick()) >= MCP342X_DEFAULT_TIMEOUT) {
            emberAfAppPrintln("MCP342X_read...return LONG_MIN...overtime, channel: %d", channel);
            return LONG_MIN;
        }

        halCommonDelayMicroseconds(250);
    }

    emberAfAppPrintln("MCP342X_read...return MCP342X_getResult()...channel: %d", channel);
    return MCP342X_getResult(channel);
}

int32_t MCP342X_readVoltage(uint8_t channel) {
    int32_t result = MCP342X_read(channel);
    uint8_t resolution = ((_config[channel] >> 2) & 0x03);//0b11
    uint8_t pga = (_config[channel] & 0x03);//0b11

    switch (pga) {
        case x1:
            pga = 1;
            break;

        case x2:
            pga = 2;
            break;

        case x4:
            pga = 4;
            break;

        case x8:
            pga = 8;
            break;
    }

    switch (resolution) {
        case _12bit:
            result /= pga;
            result *= 1000;

            break;

        case _14bit:
            result /= pga;
            result *= 250;

            break;

        case _16bit:
            result /= pga;
            result *= 62.5;

            break;

        case _18bit:
            result /= pga;
            result *= 15.625;

            break;
    }

    return result;
}


// Takes a value in mV and returns the corresponding temperature in Celsius
int32_t MCP3428Thermal_voltageToTemp(int32_t voltage) {
	// Based off of TMP 36 specification
	// Two points in the voltage-temp graph: (750mV, 25C), (1000mV, 50C).
	// So the equation is C = mV / 10 - 50
	return voltage / 10 - 50;
}


float MCP342X_getStepSize(uint8_t channel) {
  uint8_t select = _config[channel] & (MCP342X_SIZE_MASK | MCP342X_GAIN_MASK);
  return stepSizeTbl[select];
}




/*
int main() {

	MCP342X_i2cInit();

    if(MCP342X_init()){
        MCP342X_config(0, _12bit, OneShot, x2); // channel, precision, mode, PGA
        MCP342X_config(1, _18bit, OneShot, x1);
		//MCP342X_config(2, _12bit, OneShot, x2); // channel, precision, mode, PGA
        //MCP342X_config(3, _18bit, OneShot, x1);

        while (1) {
            emberAfAppPrint("ADC1 value: %ld\t", MCP342X_read(0)); // read channel 0
            emberAfAppPrint("ADC2 voltage: %ld uV\n", MCP342X_readVoltage(1)); // read voltage at channel 1
			//emberAfAppPrint("ADC3 value: %ld\t", MCP342X_read(2)); // read channel 0
            //emberAfAppPrint("ADC4 voltage: %ld uV\n", MCP342X_readVoltage(3)); // read voltage at channel 1
            halCommonDelayMilliseconds(500);//500ms
        }
    }

    emberAfAppPrintln("failed to init...");
}
*/

void mcp3428_init(void)
{
	MCP342X_i2cInit();
	halCommonDelayMilliseconds(500);

	if(MCP342X_init()) {
		MCP342X_config(0, _12bit, OneShot, x1); // channel, precision, mode, PGA
        //halCommonDelayMilliseconds(500);
        MCP342X_config(1, _12bit, OneShot, x1);
        //halCommonDelayMilliseconds(500);
    //#ifndef USE_ADC_2CH
		MCP342X_config(2, _12bit, OneShot, x1); // channel, precision, mode, PGA
        //halCommonDelayMilliseconds(500);
        MCP342X_config(3, _12bit, OneShot, x1);
        //halCommonDelayMilliseconds(500);
    //#endif

		emberAfAppPrintln("Succeeded to init mcp3428_init()...");
	}
	//emberAfAppPrintln("Invoke mcp3428_init()...");
}


void mcp3428_process(void)
{
    emberAfAppPrintln("Invoke mcp3428_process()...");

	if(!MCP3428Status){
        mcp3428_init();
        halCommonDelayMilliseconds(500);
        return;
    }

	while (1) {
		emberAfAppPrintln("ADC1 voltage: %ld uV", MCP342X_readVoltage(0)); // read channel 0
        //emberAfAppPrintln("ADC1 value: %ld uV", MCP342X_read(0));
        halCommonDelayMilliseconds(500);//500ms
		emberAfAppPrintln("ADC2 voltage: %ld uV", MCP342X_readVoltage(1)); // read voltage at channel 1
        //emberAfAppPrint("ADC2 value: %ld\t", MCP342X_read(1)); // read channel 2 // (2)
        halCommonDelayMilliseconds(500);//500ms
    //#ifndef USE_ADC_2CH
		emberAfAppPrintln("ADC3 voltage: %ld uV", MCP342X_readVoltage(2));
        //emberAfAppPrint("ADC3 value: %ld\t", MCP342X_read(2)); // read channel 2 // (2)
        halCommonDelayMilliseconds(500);//500ms
		emberAfAppPrintln("ADC4 voltage: %ld uV", MCP342X_readVoltage(3)); // read voltage at channel 3
        //emberAfAppPrintln("ADC4 value: %ld uV", MCP342X_read(3));
        halCommonDelayMilliseconds(500);//500ms
    //#endif
    	//halCommonDelayMilliseconds(500);//500ms
		break;
	}

	/*
	switch(_channel)
	{
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;

		default :
			break;
	}
	*/

}


void cli_dev_mcp3428_rst(void)
{
	emberAfAppPrintln("Reset mcp3428....");
}



#endif//#ifdef MCP3428
