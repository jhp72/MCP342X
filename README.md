# MCP342X ADC Driver

MCP342X driver based on efr32mg13p of silicon labs' chipset and gecko sdk's i2c api.

<English>
I rely on a lot of open source help in my work.
So I'm a little bit of a

I'm open-sourcing the ADC device driver in case it helps someone.

I hope it helps those who need it.

<Korean>
저도 일할 때 오픈소스 도움을 많이 받고 있습니다.
그래서 저도 미약하나마

누군가에게 도움이 될 수 있도록 adc device driver를 오픈소스로 공개합니다.

필요한 분들에게 많은 도움이 되길 바랍니다.

## Project Overview
This repository provides a C language driver for the Microchip MCP342X series of analog-to-digital converters (ADCs). It aims to offer a robust and easy-to-use interface for integrating these ADCs into embedded systems, particularly those communicating via I2C.

## Features
-   Support for various MCP342X series ADCs (e.g., MCP3421, MCP3422, MCP3423, MCP3424, MCP3426, MCP3427, MCP3428).
-   Configurable resolution and gain settings.
-   Single-shot and continuous conversion modes.
-   I2C communication interface.
-   Error handling for I2C transactions.

## Driver Details
This driver is implemented in C and designed for embedded systems. It provides functions to initialize the ADC, configure its operating parameters, and read conversion results.

## I2C Address
The MCP342X series ADCs typically have configurable I2C addresses. The base address for the MCP342X series is generally `0x68` (binary `1101000`). The last three bits (A2, A1, A0) are configurable via external pins, allowing for up to 8 different addresses.

**Default I2C Address (MCP3428):**
The MCP3428, for example, has a base address of `0x68` and its address can be modified by the A0, A1, A2 pins.
The full I2C address will be `0b1101000` + A2A1A0.

## Registers
The MCP342X ADCs are configured and read via a single configuration register and data registers.

### Configuration Register (8-bit)
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
|-------|-------|-------|-------|-------|-------|-------|-------|
| O/C   | C1    | C0    | S1    | S0    | G1    | G0    | RDY   |

-   **O/C (Output/Conversion Mode):**
    -   `0`: One-shot conversion mode
    -   `1`: Continuous conversion mode
-   **C1, C0 (Channel Selection):**
    -   `00`: Channel 1
    -   `01`: Channel 2
    -   `10`: Channel 3
    -   `11`: Channel 4 (MCP3426/7/8 only)
-   **S1, S0 (Sample Rate / Resolution):**
    -   `00`: 12-bit (240 SPS)
    -   `01`: 14-bit (60 SPS)
    -   `10`: 16-bit (15 SPS)
    -   `11`: 18-bit (3.75 SPS)
-   **G1, G0 (PGA Gain):**
    -   `00`: 1x
    -   `01`: 2x
    -   `10`: 4x
    -   `11`: 8x
-   **RDY (Ready Bit):**
    -   `0`: Data is ready
    -   `1`: New conversion in progress

### Data Registers
The conversion result is typically read as a 16-bit or 18-bit signed integer, depending on the configured resolution. The data is usually transmitted in two or three bytes (MSB first).

## Configuration Options (Enums)
The driver provides enums for easy configuration of the ADC. These typically include:

```c
// Example (actual names may vary)
typedef enum {
    MCP342X_CHANNEL_1 = 0x00,
    MCP342X_CHANNEL_2 = 0x20,
    MCP342X_CHANNEL_3 = 0x40,
    MCP342X_CHANNEL_4 = 0x60
} mcp342x_channel_t;

typedef enum {
    MCP342X_MODE_ONESHOT = 0x00,
    MCP342X_MODE_CONTINUOUS = 0x10
} mcp342x_mode_t;

typedef enum {
    MCP342X_RESOLUTION_12BIT = 0x00,
    MCP342X_RESOLUTION_14BIT = 0x04,
    MCP342X_RESOLUTION_16BIT = 0x08,
    MCP342X_RESOLUTION_18BIT = 0x0C
} mcp342x_resolution_t;

typedef enum {
    MCP342X_GAIN_1X = 0x00,
    MCP342X_GAIN_2X = 0x01,
    MCP342X_GAIN_4X = 0x02,
    MCP342X_GAIN_8X = 0x03
} mcp342x_gain_t;
```

## Key Functions
The driver typically exposes the following functions:

-   `mcp342x_init(i2c_address)`: Initializes the ADC with a given I2C address.
-   `mcp342x_configure(channel, mode, resolution, gain)`: Configures the ADC's operating parameters.
-   `mcp342x_read_raw_data(channel, &raw_value)`: Reads the raw ADC conversion result.
-   `mcp342x_convert_to_voltage(raw_value, gain, &voltage)`: Converts raw ADC data to voltage.
-   `mcp342x_start_conversion()`: Starts a new conversion in one-shot mode.

## Dependencies
-   An I2C communication library for your specific microcontroller/platform.
-   Standard C libraries (e.g., `stdint.h`).

## Usage Example
```c
#include "mcp3428.h"
#include <stdio.h> // For printf, replace with your logging mechanism

// Assume an I2C communication interface is already set up and available
// e.g., i2c_write(address, data, len), i2c_read(address, buffer, len)

int main() {
    uint8_t i2c_address = 0x68; // Example I2C address for MCP3428 with A2A1A0 = 000
    int32_t raw_adc_value;
    float voltage;

    // Initialize the ADC
    if (mcp342x_init(i2c_address) != 0) {
        printf("Failed to initialize MCP342X\n");
        return 1;
    }

    // Configure the ADC for Channel 1, One-shot mode, 16-bit resolution, 1x gain
    if (mcp342x_configure(MCP342X_CHANNEL_1, MCP342X_MODE_ONESHOT, MCP342X_RESOLUTION_16BIT, MCP342X_GAIN_1X) != 0) {
        printf("Failed to configure MCP342X\n");
        return 1;
    }

    // Start a conversion (only needed for one-shot mode)
    // In continuous mode, conversions happen automatically
    mcp342x_start_conversion();

    // Wait for conversion to complete (implement your own delay or polling)
    // For example, poll the RDY bit or wait for a fixed time based on resolution

    // Read the raw ADC value
    if (mcp342x_read_raw_data(MCP342X_CHANNEL_1, &raw_adc_value) != 0) {
        printf("Failed to read raw ADC data\n");
        return 1;
    }

    // Convert raw value to voltage
    if (mcp342x_convert_to_voltage(raw_adc_value, MCP342X_GAIN_1X, &voltage) != 0) {
        printf("Failed to convert to voltage\n");
        return 1;
    }

    printf("Raw ADC Value: %ld\n", raw_adc_value);
    printf("Voltage: %.4f V\n", voltage);

    return 0;
}
```

## License
This project is licensed under the MIT License - see the `LICENSE` file for details.