# freeRTOS_testing
## Description
This repository has been created with the purpose of studying and experimenting with the freeRTOS API. Through this project, various aspects of the freeRTOS operating system will be explored and tested, including task scheduling, inter-task communication, and synchronization. The repository will be updated regularly with new examples and code snippets that demonstrate the use of freeRTOS in various applications. This project aims to provide a valuable resource for developers looking to learn about real-time operating systems and gain experience in using the freeRTOS API.
## Queue test
In the first program, I tested the queue functionality of freeRTOS with UART communication between STM32F302R8T6 and my PC. The digital signal capture for approximately 250ms shows how the scheduler of freeRTOS manages tasks. The capture clearly displays four tasks in action: sendUSART1, receiveUSART1, Idle, and Tick. The Idle task shows us how much time the CPU is not performing any task. During the execution of the Idle task, you can potentially put your MCU in sleep mode to conserve power.

![App Screenshot](https://github.com/ArtemHW/images/blob/main/freeRTOS_test_UART_1.png)

![App Screenshot](https://github.com/ArtemHW/images/blob/main/freeRTOS_test_UART_2.png)