# DeflectionSensing

Repository for DTU NANOMADE deflection sensing.
Development of sensory feedback system capable of detecting microscopic movements during assembly of 2D heterostructures.
Hardware designed by Gustav Friis Fléron - s194009@dtu.dk / flerongustav@gmail.com
Code (as of 30/11 - 2023) written by Gustav Friis Fléron - s194009@dtu.dk / flerongustav@gmail.com

As of 30/11 - 2023 two main foldes have been added:
SOFTWARE and STM32F103_bluepill

SOFTWARE contains python code for simple COM serial data capturing. 
Goal is to write a custom .exe program to use with the STM32 system, capturing data, saving in .csv and displaying live graphing.
Currently, packaging into .exe breaks serial communication - so we'll stick to a .py script.
UI is based on PyQt5 - .ui file is included.

STM32F103_bluepill contains the stm32 project that is flashed onto the STM32F103 chip - including pin config and external libs.
When configured, the system sends out char packages/arrays through serial COM. The packages contain five data points: 2x pressure, 2x temperature and time.
Each value is comma seperated and the char array is terminated by newline "\n".




