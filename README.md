# stm32wb55_linky
how to decode linky TIC signal from your electric counter with a STM32WB55 using ADC

Most projets use an optocoupler to transform the 50KHz modulated signal form TIC (I1 / I2 pins) to a 1200 or 9600 bauds serial signal.
As I didn't want to use one, as the STM32 has a fair ADC, I planned to use the ADC to demodulated the 1200 bauds from my historical TIC.

First phase : decode the TIC signal from ADC and extract ASCII comming from the frame : complete

Seconde phase : use ZigBee to report those wattage metrics (W) and overall counter (WH) to my coordinator (zigbee2mqtt) : ongoing

Third phase : add some more current coil on my distribution board and ADC convertion to get individual amperage from each circuit and report it to zigbee


*Documentation* : 
- Linky TIC interface : https://www.enedis.fr/media/2035/download
- STM32WB reference manual : https://www.st.com/en/microcontrollers-microprocessors/stm32wb-series/documentation.html
- The board I used (5$ on aliexpress) : https://www.aliexpress.com/item/1005007291039616.html
- Its schematic here : https://github.com/WeActStudio/WeActStudio.STM32F4_64Pin_CoreBoard/blob/master/Hardware/WeAct-STM32F4_64PIN-CoreBoard_V10%20SchDoc.pdf


*Schematic* : inside [schematic](schematic) folder

*Design* : some explanation about algorithms used inside program in [design](design) folder

*Code* : inside [source](source) folder (c code and STM32cube .ioc file)
