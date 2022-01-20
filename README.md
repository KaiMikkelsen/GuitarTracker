# Pong, Ping

## Problems I have faced:

- STM32 Board not recognized as USB Device
- Solution: Had to use a different cable, (the one for my wireless beats headphones)

- Serial communication not working
- Solution: Created a fresh new project and realized UART was already configured automatically with the chip, so restarting it was just wasting my time.

- Struggled to get values going from top of the screen to the bottom to represent the user paddle
- Solution: Looked at logic more closely

- I2C communication with screen not working
- Solution: Having the potentiometer on the same power as the screen was messing with it, my knowledge of electronics is lacking clearly...
- Solution pt2: Figured out the potentiometer I was using to control user movement was shorting out the board or something, switched to a better joystick controller and am using the other side of the board for ADC which is working fine for now...

- Such a small limited number of pixels on the screen, and the minimum ball speed is 1pixel per frame, and i dont want to use floating point numbers as its not recommended for microcontroller programming, how can i do this with ints, while still creating playable movement in the x/y directions.
- Solution:

- Glitchy Player Paddle when moving joystick is pushed up
- Solution: Quite sure this was caused by the ADC getting the almighty power of 5v when the joystick is moving up and fully open, which was causing some sort of communication issue. Fixed this by running the joystick on a 3v power supply which fixed the issue.

- UART with ESP8266 not working
- Solution: None yet smh



## Hours

January 5th
- 2 Hours: Trying to get it plugged in unsuccessfully, experiemnting with STLink

January 6th
- 2 Hours: Got it connecting with a better cable, reading adc values from a potentiometer

January 7th
- 2 Hours: Trying and failing to get serial communication working, sound sensor not really working

January 8th
- 4 hours: Serial communication solved! looking into using a microphone IC instead of piezo buzzer as my electronics knowledge is lacking

January 9th
- 8 hours: Struggled to get acurate readings on ADC, ended the day getting a user bar floating up and down depending on joystick position, took me much too long

January 10th
- 1 hour: Created a display thread, added a ball

January 11th
- 2 hours: Got basic X axis collision working and working on Y angles to add spin to the ball

January 12th
- 2 hours: Back and forth gameplay working, next to implement is the score system

January 13th
- 1 hour: Improved back and forth gameplay, finding and fixing bugs

January 14th
- 4 hours: Huge Friday spend... Debugging, Got a fully functional game with minimal bugs working

January 15th
- 4 hours: Improved score system, AI and gameplay. Looking into using the esp8266 for wifi connectivity

January 16th
- 5 hours: Trying to get uART communication with esp866 board, not having much luck..

- January 18th
- 1 Hour: More uart stuff, tried with a a new empty project still no luck.

January 19th
- 1.5 hours: Still trying to get uart working, logic analyzer would be nice, going to go back and make sure I understand how UART actually works like I think

## Resources

Digikey stm32 tutorials: https://www.youtube.com/watch?v=EsZLgqhqfO0&ab_channel=Digi-Key
Controllers tech on using OLED screen: https://www.youtube.com/watch?v=M5ddTjrcvEs&ab_channel=ControllersTech
StackOverflow user on how to calculate y collision angles:https://gamedev.stackexchange.com/questions/4253/in-pong-how-do-you-calculate-the-balls-direction-when-it-bounces-off-the-paddl
