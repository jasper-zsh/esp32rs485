# ESP32 RS485 Project - Context Information

## Project Overview

This is an ESP-IDF based project for an ESP32-S3 microcontroller that implements a dual RS485 communication system with BLE connectivity and power management features. The project is designed to work in automotive environments where power consumption and communication reliability are critical.

## Hardware Spec
@./designs/hardware-spec.md

### Key Features

1. **Dual RS485 Communication Channels**:
   - Channel 1 (UART1): GPIO12 (TX), GPIO13 (RX)
   - Channel 2 (UART2): GPIO10 (TX), GPIO11 (RX)
   - Configurable protocols: RAW, Modbus RTU, Modbus ASCII

2. **BLE Connectivity**:
   - Custom BLE service (UUID: 0xFF00) with multiple characteristics
   - Device pairing and authentication mechanism
   - Configuration management via BLE
   - Bidirectional data transfer between BLE and RS485 channels

3. **Power Management**:
   - Ultra-Low Power (ULP) RISC-V coprocessor for voltage monitoring
   - Deep sleep mode when voltage drops below threshold (13V)
   - Automatic wake-up when voltage returns to normal

4. **User Interface**:
   - WS2812 LED strip (GPIO21) for status indication
   - Factory reset button (GPIO4) with 5-second hold detection

5. **Storage**:
   - NVS (Non-Volatile Storage) for configuration persistence

## Project Structure

```
├── CMakeLists.txt                 # Project-level CMake configuration
├── main/
│   ├── CMakeLists.txt             # Main component CMake configuration
│   ├── main.c                     # Main application entry point
│   ├── power_management.c/h       # Power management implementation
│   ├── ble.c/h                    # BLE functionality implementation
│   ├── ulp/
│   │   └── power.c                # ULP RISC-V program for voltage monitoring
├── designs/
│   ├── ble.md                     # BLE communication protocol design
│   └── power_management.md        # Power management strategy design
├── dependencies.lock              # Project dependencies lock file
├── .devcontainer/                 # Docker container configuration for development
└── README.md                      # Project documentation
```

## Technology Stack

- **Framework**: ESP-IDF (Espressif IoT Development Framework) v5.5.0
- **Microcontroller**: ESP32-S3
- **Components**:
  - `espressif/esp-modbus` v2.1.0
  - `espressif/led_strip` v3.0.1
  - Bluetooth LE stack
  - UART drivers
  - ADC drivers
  - ULP RISC-V coprocessor

## Building and Running

### Prerequisites

- ESP-IDF v5.5.0 or compatible version
- Docker (optional, for containerized development)
- ESP32-S3 development board

### Build Process

1. Source ESP-IDF environment:
   ```bash
   . $IDF_PATH/export.sh
   ```

2. Configure the project (optional):
   ```bash
   idf.py menuconfig
   ```

3. Build the project:
   ```bash
   idf.py build
   ```

4. Flash to the device:
   ```bash
   idf.py flash
   ```

5. Monitor serial output:
   ```bash
   idf.py monitor
   ```

### Development Container

This project includes a devcontainer configuration for consistent development environment setup:
- Based on `espressif/idf:latest` Docker image
- Includes ESP-IDF extension for VS Code
- Automatically sources ESP-IDF environment

## Key Implementation Details

### BLE Communication Protocol

- Service UUID: 0xFF00
- Settings Characteristic (0xFFF0): Configuration management
- RS485 Channel 1 Characteristic (0xFFF1): Data transfer for channel 1
- RS485 Channel 2 Characteristic (0xFFF2): Data transfer for channel 2

#### Configuration Management
- Device pairing/unpairing
- RS485 channel mode configuration
- Paired device list management

#### Security
- Only paired devices can access most BLE features
- First device to pair becomes the "owner" and can add/remove other devices

### Power Management Strategy

- Uses ULP RISC-V coprocessor for continuous voltage monitoring
- Enters deep sleep when voltage drops below 13V (automotive application)
- Wakes up every second to check voltage during sleep
- Automatically resumes normal operation when voltage returns to normal

### RS485 Channels

- Two independent RS485 channels with configurable protocols
- Data can flow bidirectionally between BLE and RS485
- Mode configuration stored in NVS for persistence

## Development Guidelines

### Code Organization

- Modular design with separate modules for BLE, power management, and main application
- ULP code separated into its own directory
- Configuration constants defined at the top of each file
- Comprehensive logging with appropriate log levels

### Testing

- Manual testing via BLE connection
- Serial logging for debugging
- LED status indicators for quick visual feedback

### Adding New Features

1. Create new source/header files in the main directory
2. Update `main/CMakeLists.txt` to include new files
3. Register any new components with the IDF framework
4. Add appropriate error handling and logging
5. Update documentation in the `designs/` directory

## Debugging and Troubleshooting

### Common Issues

1. **BLE Connection Problems**:
   - Check device pairing status
   - Verify that the device is advertising
   - Ensure sufficient power supply

2. **RS485 Communication Issues**:
   - Verify baud rate settings
   - Check wiring connections
   - Confirm protocol mode configuration

3. **Power Management Issues**:
   - Monitor serial logs for ULP activity
   - Check voltage readings
   - Verify deep sleep/wake-up cycles

### Useful Commands

- Clean build: `idf.py clean`
- Full rebuild: `idf.py fullclean build`
- Erase flash: `idf.py erase-flash`
- View memory usage: `idf.py size`
