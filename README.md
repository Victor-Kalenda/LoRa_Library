# LoRa_Library
Library implementation of Semtech's sx126x_driver for Arduino microcontrollers

With LoRa_Library, users will have access to the full potential of the sx126x. The purpose of this library is 
to help developers research the sx126x transceiver with low cost equipment and little to no unnecessary development 
costs. All code was based on the sx126x_driver provided by Semtech, with additional functionality solely based on 
the sx126x datasheet. Upon analysis of LoRa_Library.cpp, users will be able to reference back to specific sections 
of the sx126x datasheet for all abstract function calls. Allowing developers to build a strong foundational 
understanding for what is required to make the sx126x operate smoothly. LoRa_Library does not support FHSS 
modulation as it is not a transceiver-to-transceiver communication technique for the sx126x.

LoRa_Library is an excellent tool for developing your own wireless communication protocol due to it's efficient
space and time complexity. It is strongly recommended that users get familiar with the sx126x_Testing script to 
understand the main functions of the sx126x, and the behavour of the sx126x in any given state. All information
regarding the use of the sx126x_Testing script can be found on Testing.pdf.

For users who are looking for the bare minimum required to start development of an sx126x based communication
protocol, it is advised to program your protocol using the sx126x_driver provided by Semtech on the LoraNet 
github repository.


Recommended Hardware and Design Considerations

Dorji Applied Technologies have an excellent FCC compliant sx126x transceiver module which can be purchase along 
with the DAD06 development board for testing sx126x transceivers with an Arduino Uno. Alternatively, users can 
design their own PCB with an ATMEGA328, please be advised that sx126x transceivers are only 3.3V tolerant. Therefore, 
logic level conversion from 5V to 3.3V is required to ensure the sx126x transceiver is not damaged. A workaround to 
this issue can also be to design an ATMEGA328 to run on a 3.3V power supply. For this to work, the microcontroller 
will require an 8MHz external crystal oscillator according to the ATMEGA328 datasheet for operating at 3.3V, along 
with an FTDI converter to communicate with the microcontroller.

![IMG_3292](https://github.com/Victor-Kalenda/LoRa_Library/assets/90730727/19b578fc-3888-47d9-b945-0d6798fc4d3c)



