# Modulation of TIC

The signal of TIC is a 50KHz sin wave modulated by a serial signal at 1200 bauds (for historical format, and 9600 bauds for the new one).

In order to decode it using ADC, we need to sample the sin wave at a higher frequency. We chose 150KHz (3x the base frequency) to be able to have at least 2 measurements not null.

![3 pts](./3pts_per_sin.svg)

For 1200 bauds, we will deal with 150/1.2 or **125** samples of the sin wave per bit.

For 9600 bauds, it will be 150/9.6 or **15.6** samples

It seems enough to get a fair idea of bits hidden behind the sin waves

# ADC, sampling, converting

The STM32's ADC is able to achieve 12 bits measurements at frequencies higher than 1MHz. We don't need that much. 150KHz is 10% of the capacity and 8 bits is enough to get an idea of bearer presence or not.

ADC use a specific clock than can be derived from internal CPU clock or an alternative one based on a PLL (choice we made)

Sampling (the time the switch opens to sync its capacitance before converting) can be ajusted to a various range of clocks ticks from the PLL entry frequency (and you can choose different ones for differents entries if needed)
![sampling](./adc_sampling_periods.PNG)

So 8 bits conversion takes only 8.5 clock cycles
![clock tree](./8bits_conv_time.PNG)

So as ADC PLL clock has constraints (of being above 1MHz or such), 47.5 cycles for sampling and 8.5 for ADC conversion equals 56 cycles. So 150KHz times 56 will bring me to 8.4MHz for my PLL. But I can choose a higher frequency and use a divider :

![clock tree](./sampling_clocks.PNG)

I choose /4 that brings me to a more PLL friendly frequency of 33.6MHz. As the input frequency of my PLL will be 32MHZ / 2 /4 or 4MHZ, the division of 33.6 by 4 is the fraction 42/5 I'm done.

![clock tree](./adc_clock.PNG)

So to sum up, I maximise my sampling time on a ratio of 47.5/56 or I will be samping 85% of the time and converting 15%. So when my 50KHz sin wave will be cut in 3 parts, I will have at least one 100% positive or negative.

![clock tree](./3samp_conv.svg)

# extracting bits

The signal from the linky is zeroed with a scale of two resistor to half of 3.3V and reduced to fit in the 0 - 3.3V range of the ADC. We get something like that :

![clock tree](./1v65.svg)

On the ADC, 0V equals value 0x00 and 3.3V 0xff

## shaping

We need to make some calculation on the raw signal :
- average on a long period, as we have part of signal with raw bearer (the 50KHz sin) and other with a very flat sin wave (of few milivolts)
- then keeping the amplitude of signal compared to the average line (to get energy of signal)
- then averaging the energy between 0 and 1 train of bits
- then filtering measurements on 3 successive 150KHz sample to get an actual idea if we have some energy on the 50KHz period (we are not constraint by the signal sync because we will have many period of the bearer present or absent)

![clock tree](./averages.svg)

## alternance (spacing) measurements