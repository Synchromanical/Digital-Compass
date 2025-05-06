# Digital-Compass
STM32CubeIDE Compass & Display Firmware project for STM32F4 series that reads magnetometer and accelerometer data, computes heading, displays it on an I²C LCD, and shows direction via GPIO pins to external LEDs using FreeRTOS tasks. (Not fully working)

## Key Features
### FreeRTOS-Based Multithreading
- **Implements two concurrent tasks**
  - sensorTask: reads LSM303AGR accelerometer and magnetometer data, computes pitch/roll and heading, and sends the heading value to a message queue.
  - ledTask: waits on a semaphore, retrieves the latest heading from the queue, and lights one of eight LEDs to indicate the compass direction.

- **LSM303AGR Sensor Initialization & Data Processing**
  - Configures the LSM303AGR magnetometer for continuous conversion at 15 Hz and the accelerometer at 50 Hz.
  - Applies calibration offsets (`MAG_X_OFFSET`, `MAG_Y_OFFSET`) and tilt thresholds to ensure valid heading only when the board is level.

- **I²C-Driven HD44780-Compatible LCD Interface**
  - Low-level routines (`LCD_SendInternal`, `LCD_Init`, `LCD_Clear`, `LCD_SetCursor`, `LCD_SendString`) to drive a 4-bit LCD over I²C at address `0x27<<1`.
  - Displays raw X/Y magnetometer readings and the computed heading in real time.

- **GPIO LED Compass Indicator**
  - Defines eight GPIO pins (LD7–LD14) to represent 45° compass octants.
  - Dynamically lights the LED corresponding to the current heading. 

- **Peripheral Initialization**
  - Sets up all required peripherals via STM32Cube-generated `MX_*_Init()` functions:
  - I²C1 & I²C2 for sensor and LCD
  - SPI1 & I²S3 for future audio/expansion
  - RTC for timekeeping
  - GPIO for LEDs and interrupts
  - System clock configuration for HSI/LSI with PLL at 100 MHz 

- **Thread-Safe I²C Access**
  - Uses an RTOS mutex (`i2cMutexHandle`) to protect I²C transactions across tasks. 
