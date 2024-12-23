
# CN0267 - HART Field Device Demo

A simple HART field device implementation based on [ADI DEMO-AD5700D2Z Board](https://www.analog.com/en/resources/reference-designs/circuits-from-the-lab/cn0267.html)

![Image](https://www.analog.com/en/_/media/analog/en/reference-circuits/images/cn0267-02-1024.gif?w=900&thn=1&rev=c819e7ebddaf41e794ced6812e3c20be)

# Hardware description

 -> Sensor input: Only the on-board RTD sensor is used for PV 
 
 -> MCU: ADuCM360 with 24-bit sigma-delta ADC
 
 -> HART Modem: AD5700

 -> DAC + Analog output: AD5421 (4~20mA)
 
# Software description

A complete EWARM project based on [CN0267 Design Support Package](https://www.analog.com/media/en/reference-design-documentation/design-integration-files/CN0267-DesignSupport.zip)

Besides the basic transmitter functions, it implements a simple HART v7 communication protocol of field device, with all the universal commands supported.

-- src

 |-- main.c: basic transmitter functions and all the universal commands

 |-- HART_AD5700: a simple HART v7 slave stack

 |-- FlashEraseWrite: Flash erase & write for NVM data

 |-- ADC_RTD: RTD sampling & calculation
 
 |-- AD5421: Analog output

## Authors

- [@wang58jun](https://github.com/wang58jun)
