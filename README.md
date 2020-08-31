# GSoC 2020: Linux IIO Driver for LTC2376-16

- Meenal Parakh

## Organization 

The Linux Foundation 

## Mentors

- Darius Berghe
- Dragos Bogdan

## Code

This repository is forked from [ADI's Linux](http://github.com/analogdevicesinc/linux.git) and [this branch](https://github.com/meenalparakh/linux/tree/ltc2376_1) is used for submitting pull requests.

## Project Summary

The project was to build a linux IIO driver for LTC2376-16 and test it on hardware 
and possibly send the code upstream for review. The current driver provides functionality for reading raw values, with corresponding scale and offset and a data ready triggered buffer support. The buffer, when enabled, fills itself 
at the frequency of the PWM used for the device as conversion signal. A tutorial/report on driver development can be found 
[here](https://drive.google.com/file/d/1lqPyOmodl7Mw3s4rtFXzEHlTutADTKb8/view?usp=sharing).

### Left to do 

- Fix the code based on the feedback from the ADI members. 
- Send the code upstream for review and fix the code as needed.
- A possible extension would be to make the driver functional for closely 
related devices like LTC2376-18 and LTC2379.

### Pull requests 

- [LTC2376 driver skeletal](https://github.com/analogdevicesinc/linux/pull/1052) (Approved) -  Adds a basic driver skeleton for LTC2376 (single channel, 16-bit SAR ADC) along with its device tree overlay for Raspberry Pi3 and device tree binding documentation in yaml. Consists of commits as follows - 
    - [boot: dts: overlays: Add dtoverlay for LTC2376](https://github.com/meenalparakh/linux/commit/4c8a8260bc714ac02b7f7e846d1b95aaa0df1813)
    - [dt-bindings: iio: adc: Add DT binding for LTC2376](https://github.com/meenalparakh/linux/commit/d90db2a6096304fcbf5b34b7aa2379c09463d5eb)
    - [arch: arm: configs: Enable LTC2376 in defconfig](https://github.com/meenalparakh/linux/commit/36fe23c279b9505705001687edf6d8ce13bfed4d)
    - [iio: adc: Add driver for LTC2376](https://github.com/meenalparakh/linux/commit/0d8c12d5475347ce8e2acdac49ebd5e609ad69d4)
    
- [LTC2376 channel and buffer support](https://github.com/analogdevicesinc/linux/pull/1133) (Currently under review) - Adds basic read raw functionality and buffer support. The frequency of the PWM can be varied and is used as a conversion signal for the device. The falling edge on the busy signal from the device provides hardware trigger for the buffer. 

### Acknowledgements

I would like to thank Darius Berghe and Dragos Bogdan for providing guidance 
and invaluable suggestions crucial for proceeding in the project. I would also 
like to thank Mark Thoren for providing useful tools and gadgets for testing 
the device driver. I also thank Nishant Malpani for sharing his knowledge on 
driver development and ADI members for providing suggestions and reviewing 
the code.
