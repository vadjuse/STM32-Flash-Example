# STM32-Flash-Example
This main.c file containes simple example of STM32 internal flash memory working sequence.
You need to configure clock settings via registers or by using CubeMX. 
After that you need to know two things:
1. (If you use a nucleo DevBoard) USART connected to Virtual COM Port. Read User Manual of your MCU and reconfig PORT and corresponding pins if needed.
2. Amount of internal Flash memory in your MCU. For example my STM32F334R8 has 32 pages of Flash, so to make things safely I use last (31) page. You should set your value. Also your check refmanual for page erase procedure because it can be different. 


The main program just accumulates received bytes to line[] string and then you send '\r' sends whole string back to USART and save this string to Flash in blocking manner.
Then started main program send to USART contants of last flash page if it's not empty.
