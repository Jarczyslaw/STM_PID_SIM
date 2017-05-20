PID regulator and dynamic system simulator on STM32
=================

Simple Coocox project implementing PID regulator on STM32F4DISCOVERY board (with STM32F407 uc). 

Features:
- runs with simple RC circuit as dynamic system
- simulates discrete and continous second order inertia system
- uses VCP to send all neccessary data to PC
- implements PID with anti-windup 
- implements relay
- uses ADC with DMA to get voltages from RC circuit
- uses PWM as control value
- simple button debouncing

Check this video to see how it works: [STM32F4 Discovery + NI LabView ](https://www.youtube.com/watch?v=DZl_jsHc-Uw)

To setup project with VCP see here: [STM32DiscoveryVCP](https://github.com/xenovacivus/STM32DiscoveryVCP)