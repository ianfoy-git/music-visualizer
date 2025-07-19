# Music Visualizer with Autonomous Line Following

This project was developed as part of the Embedded Systems Design course at the University of Michigan. It integrates digital signal processing (DSP) for real-time music visualization with autonomous robotic navigation using a Zumo robot and STM32 microcontroller.

## Features

- Real-time LED equalizer that visualizes music input
- LCD display for system feedback
- Autonomous line-following robot using QTR-8RC reflectance sensor array
- Wireless communication between devices using XBee modules
- H-Bridge motor control for differential drive
- PWM, GPIO, and timer-based driver development

## Technologies Used

- C and ARM Assembly
- STM32 Nucleo board
- QTR-8RC reflectance sensors
- XBee wireless modules (UART)
- H-Bridge motor driver
- Bare-metal firmware development
- STM32CubeIDE and HAL libraries

## Project Structure

├── Core/ # Main application code
├── Drivers/ # HAL and sensor driver files
├── Inc/ # Header files
├── Src/ # Source code implementations
├── .project, .cproject # STM32CubeIDE metadata
└── README.md


## Setup and Usage

1. Clone this repository.
2. Open the project in STM32CubeIDE.
3. Connect the STM32 Nucleo board to the Zumo robot and peripherals.
4. Flash the firmware onto the board.
5. Play music through the input circuit to activate visualization and robot behavior.

## Author

Ian Foy  
Email: ianfoy@umich.edu  
LinkedIn: [https://www.linkedin.com/in/ian-foy-778439263/](https://www.linkedin.com/in/ian-foy-778439263/)  
Portfolio: [https://ianfoy-git.github.io/ianfoy.github.io/](https://ianfoy-git.github.io/ianfoy.github.io/)

