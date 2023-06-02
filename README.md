# STM32-Flash-Test
This main.c file containes simple example of working with STM32 internal flash memory.
You need to configure clock settings via registers or by using CubeMX. 
After that you need to know two things:
1. (If you use a nucleo DevBoard) USART connected to Virtual COM Port. Read User Manual of your MCU and reconfig PORT and corresponding pins if needed.
2. Amount of internal Flash memory in your MCU. For example my STM32F334R8 has 32 pages of Flash, so to make things safely I use last (31) page. You should set your value.


The main program just accumulates received bytes to line[] string and then you send '\r' sends whole string back to USART and save this string to Flash in blocking manner.
Then started main program send to USART contants of last flash page if it's not empty.
