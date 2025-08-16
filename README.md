# MCP342X
mcp342x driver based on efr32mg13p, sliconlabs' chiset and gecko sdk's i2c api.


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

## Project Structure and File Descriptions

This project, `MCP342X`, provides a driver for the MCP342X series of Analog-to-Digital Converters (ADCs), specifically tailored for use with Silicon Labs' EFR32MG13P chipset and leveraging the Gecko SDK's I2C API.

Here's a detailed breakdown of the files and directories within this repository:

### Files:

*   **`mcp3428.c`**:
    This is the primary C source file containing the implementation of the MCP342X ADC driver. It includes functions for initializing the ADC, configuring its operating parameters (e.g., resolution, gain, conversion mode), and reading conversion results. This file directly interacts with the I2C bus to communicate with the MCP342X device. It is designed to be integrated into embedded systems projects using the EFR32MG13P microcontroller.

*   **`mcp3428.h`**:
    This is the header file corresponding to `mcp3428.c`. It declares the data structures, function prototypes, and macros necessary for interfacing with the MCP342X driver. Any other C file that needs to use the MCP342X driver functions will include this header file. It defines the public API of the driver.

*   **`README.md`**:
    This file provides a general overview of the `MCP342X` project, including its purpose, a brief description of the driver, and information about its open-source nature. It also contains instructions or general guidance for users. This file is written in Markdown format for easy readability on platforms like GitHub.

*   **`LICENSE`**:
    This file contains the MIT License under which this project is distributed. It specifies the terms and conditions for using, copying, modifying, and distributing the software. It ensures that the software remains open source while protecting the rights of the copyright holder. The copyright is held by Jaehong Park <smilemacho@gmail.com>.

### Directories:

*   **`.git/`**:
    This directory is a hidden folder created and managed by Git, the version control system. It contains all the necessary information for the local Git repository, including commit history, remote repository addresses, branches, and configuration settings. It allows developers to track changes, revert to previous versions, and collaborate on the project. You typically do not need to interact with this directory directly.

## Key Functions and Their Descriptions

This section details the main functions provided by the `mcp3428` driver, outlining their purpose and functionality.

### Functions in `mcp3428.h` (Public API):

*   **`void mcp3428_init(void)`**:
    Initializes the MCP3428 driver and the underlying I2C communication. This function should be called once at system startup to prepare the ADC for operation. It also configures the initial settings for the ADC channels.

*   **`void mcp3428_process(void)`**:
    Handles the continuous processing and reading of ADC values from the configured channels. This function is typically called periodically within a main loop or a task scheduler to acquire new ADC data.

*   **`void cli_dev_mcp3428_rst(void)`**:
    A utility function, likely intended for command-line interface (CLI) usage, to reset the MCP3428 device. This can be useful for reinitializing the ADC during debugging or specific operational scenarios.

### Functions in `mcp3428.c` (Internal/Helper Functions):

These functions are part of the driver's internal implementation and are typically not called directly by external modules, but they are crucial for the driver's operation.

*   **`void MCP342X_i2cInit(void)`**:
    Initializes the I2C communication interface used to communicate with the MCP342X ADC. This sets up the I2C peripheral with the necessary clock frequency and other parameters.

*   **`bool MCP342X_init(void)`**:
    Performs the initial setup of the MCP342X device. This includes setting default configurations for the ADC channels (e.g., continuous mode, 12-bit resolution) and verifying I2C communication with the device.

*   **`bool MCP342X_config(uint8_t channel, Resolution res, Conversion mode, PGA gain)`**:
    Configures a specific channel of the MCP342X ADC. Parameters allow setting the input `channel`, `resolution` (e.g., 12-bit, 14-bit), `conversion mode` (one-shot or continuous), and `Programmable Gain Amplifier (PGA)` gain (e.g., 1x, 2x, 4x, 8x).

*   **`bool MCP342X_newConversion(uint8_t channel)`**:
    Initiates a new analog-to-digital conversion on the specified `channel`. This is particularly relevant when the ADC is configured in one-shot conversion mode.

*   **`int8_t MCP342X_isConversionFinished(uint8_t channel)`**:
    Checks the status of the ADC conversion for a given `channel`. It returns 1 if the conversion is complete and the data is ready, 0 if not ready, and -1 in case of an error.

*   **`int32_t MCP342X_getResult(uint8_t channel)`**:
    Retrieves the raw digital result of the ADC conversion for the specified `channel`. The result is returned as a 32-bit integer, with its interpretation depending on the configured resolution.

*   **`int32_t MCP342X_read(uint8_t channel)`**:
    Performs a complete read operation for a specified `channel`. If the ADC is in one-shot mode, it initiates a new conversion, waits for it to complete, and then retrieves the raw digital result.

*   **`int32_t MCP342X_readVoltage(uint8_t channel)`**:
    Reads the ADC value from the specified `channel` and converts it into a voltage reading (in microvolts). This function takes into account the configured resolution and PGA gain to provide an accurate voltage representation.

*   **`float MCP342X_getStepSize(uint8_t channel)`**:
    Calculates and returns the step size (Least Significant Bit, LSB value) for the given `channel` based on its current configuration (resolution and gain). This value represents the voltage equivalent of one digital count.

*   **`int32_t MCP3428Thermal_voltageToTemp(int32_t voltage)`**:
    A specialized function that converts a given voltage reading (in mV) into a temperature in Celsius. This is likely used in conjunction with a specific thermal sensor (e.g., TMP36) whose output voltage is proportional to temperature.

*   **`uint32_t MCP342x_Resolution_getConversionDelayTime(uint8_t resolution)`**:
    Provides the typical conversion delay time in microseconds for a given ADC `resolution`. This can be used to implement appropriate delays when waiting for conversions to complete.

*   **`bool I2C_generalCallReset(void)`**:
    Sends an I2C General Call Reset command (0x06) to all devices on the I2C bus. This can be used to reset the state of all I2C slave devices.

*   **`bool I2C_generalCallLatch(void)`**:
    Sends an I2C General Call Latch command (0x04). The specific effect of this command depends on the I2C slave devices.

*   **`bool I2C_generalCallConvert(void)`**:
    Sends an I2C General Call Convert command (0x08) to all devices on the I2C bus. This can be used to initiate a conversion on all connected ADCs simultaneously.

*   **`void mcp342x_i2c_init(void)`**:
    An internal helper function that performs the low-level initialization of the I2C peripheral. It is called by `MCP342X_i2cInit`.
