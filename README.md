# Digital clock

## Table of contents:

- [Introduction](#introduction)
- [Project Features](#project-features)
- [Project Structure](#project-structure)
-

### Introduction
Thisproject Implement  Stop Watch system.
>
*First: The components*
>
    1. Use ATmega32 Microcontroller with frequency 1Mhz.
    2. Configure Timer1 in ATmega32 with CTC mode to count the Stop Watch time.
    3. Use six Common Anode 7-segments.
    4. Connect 7447 decoder 4-pins to the first 4-pins in PORTC.
    5. Use first 6-pins in PORTA as the enable/disable pins for the six 7-segments.
>
*Second: The Functionality*
>
    1. Stop Watch counting should start once the power is connected to the MCU.
    2. Configure External Interrupt INT0 with falling edge. If a falling edge detected the Stop Watch time should be reset.
    3. Configure External Interrupt INT1 with raising edge.If a raising edge detected the Stop Watch time should be paused.
    4. Configure External Interrupt INT2 with falling edge.If a falling edge detected the Stop Watch time should be resumed.
>

### Project Features



### Project Structure
The Application is built using:
  - C programing language
  - Proteus for simulation


### Submitted to:

-  Eng. Mohamed Tarek
 
