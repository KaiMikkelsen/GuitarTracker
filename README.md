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
- Solution pt2: Figured out the potentiometer I was using to control user movement was shorting out the board or something, switched to a better joystick controller and am using the other side of the board for ADC which is working fine for now..


## Hours

January 5th
- 2 Hours: Trying to get it plugged in unsuccessfully, experiemnting with STLink

January 6th
- 2 Hours: Got it connecting with a better cable, reading adc values from a potentiometer

January 7th
- 2 Hours: Trying and failing to get serial communication working, sound sensor not really working.

January 8th
- 4 hours: Serial communication solved! looking into using a microphone IC instead of piezo buzzer as my electronics knowledge is lacking

January 9th
- X hours: Struggled to get acurate va


