# EdgeFlight OBC - STM32H7 On-Board Computer

## Overview
On-Board Computer (OBC) firmware for STM32H743 microcontroller, supporting ADCS (Attitude Determination and Control System) and camera interface capabilities.

## Features
- Multiple IMU sensor support for attitude determination
- FDCAN communication interface
- UART/USART interfaces
- I2C sensor communication
- Camera interface support
- MessagePack serialization for efficient data transmission

## Hardware
- **MCU**: STM32H743
- **Development Board**: OBC Development Board
- **Supported Sensors**:
  - GY-85 (9-axis IMU)
  - HMC6343 (Digital compass with tilt compensation)
  - MPU9250 (9-axis MotionTracking device)
  - SparkFun sensors (ISM330DHCX, MMC5983MA)

## Development Environment
- **IDE**: Keil MDK-ARM
- **HAL**: STM32H7xx HAL Driver
- **CMSIS**: ARM CMSIS for STM32H7

## Project Structure
```
FlightController-STM32H7/
├── Inc/                 # Header files
├── Src/                 # Source files
├── Sensors/            # Sensor driver implementations
├── Services/           # MessagePack library
├── Serialization/      # Data encoding/serialization
├── Drivers/            # STM32 HAL and CMSIS drivers
└── MDK-ARM/           # Keil project files
```

## Building the Project
1. Open `FlightController-STM32H7/MDK-ARM/Test.uvprojx` in Keil MDK-ARM
2. Build the project (F7)
3. Flash to target device

## Notes
- HAL and CMSIS drivers are included in the repository for convenience
- Original firmware provided by OBC board manufacturer
- Consider migrating to git submodules for HAL/CMSIS in future updates

## License
This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.