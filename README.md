# Megahertz class arbitrary signal generation

The main branch has the base code that generates a signal at the frequency selected in 'main.c'

Select BASE_PIN as your LSB (least significant bit), and the MSB should be BASE_PIN + 8. Wire your R2R dac to this range of GPIO pins.

## R2R DAC Ladder

I use the [Bournes
4310R-R2R-102](https://www.mouser.com/ProductDetail/Bourns/4310R-R2R-102?qs=ti0co70x5DSPl2HS5sX%252B3A%3D%3D&srsltid=AfmBOorSxPF-ga_wRNfuIj7D_fiogMa9FJ9P8adYRVjEMQbmI8hFbfLk) R2R DAC Ladder. This is a lower impedance module which helps with amplitude at higher frequencies. The center 8 pins need to be connected in sequence to the selected GPIO pins, with the MSB going to pin 2.

Pin 10 goes to ground, pin 1 outputs the analog wave form.

## Compiling

Install the C/C++ SDK for the raspberry pi pico. I also use the VS Code plugins to make the process of compiling and flashing easier.

## Viewing the output

You'll need a high bandwidth oscilloscope to see the output. Usable waveforms will cap out at about 30-40MHz, though with frequencies that are integer dividers of the clock rate you should be able to hit clockrate/2 - 150MHz with the stock overclock.

## Radio branch

`git checkout radio`

Will check out the radio branch. Select a frequency in the AM radio range 530kHz to 1710kHz. The code has a short clip of Beethoven encoded as 8kHz PCM samples. You can of course generate your own clip and then encode it with the command:

`xxd -i dadadadum8k.wav > samples.c `

The range is a few inches. Dangle a wire out of pin 1 of the DAC as an 'antenna'. I did play around with some amplification and antenna stages but nothing that worked well enough to recommend. More research would be needed to extend this to usable ranges, say if you wanted to transmit inside your house.
