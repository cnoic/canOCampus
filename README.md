# canOCampus
Connect to an array of sensors through a Can Bus
## Introduction
The goal of this projet is to create an array of sensors, sized to fill a building with the minimal ammount of installation and the best versatility possible.
The system consists in a Master which is essentially a RPi with a Can controller connected via SPI. This Master controls End Devices, (stm32 "bluepill") which can read sensors and control acuators.
For a better versatility and to make this system transparent in use, the RPi has libraries to read and write GPIOs, use Serial Buses, use I²C and SPI remotely.


## Protocol
### Master
#### SETPIN (0x01)
`[0b00100000 + *numPin*] [*valPin* >> 8] [*valPin* &0xFF]`

Ask the end device to change the value (*valPin*) of a GPIO (*numPin*)
- The pin must be configured as an output
  - Else the end device will return error code **R_FORBIDDEN**
- *valPin* is on 2 char to enable 10 bit resolution on PWM if needed in future, for now the 1st char can be set to 0x00 as the analogWriteResolution is on default (8 bit)

example :\
`[0x21,0x00,0x5F] -> set Pin 1 to 95/255`

#### GETPIN (0x02)
`[0b01000000 + *numPin*]`

Ask the end device to return the value of the GPIO (*numPin*)
- The pin must be configured as an Input
  - Else the end device will return error code **R_FORBIDDEN**
- The end device will respond with a **READPIN (0x02)** packet

example :\
`[0x41] -> Get the value of Pin 1`

#### SETPINMODE (0x03) *(To modify)*
`[0b01100000 + *numPin*] [*pinMode*] [*t_low* >> 0x08] [*t_low* & 0xFF] [*t_high* >> 0x08] [*t_high* & 0xFF] [*inverted*]`

Set the mode of a specific GPIO (*numPin*) on the end device
If *numPin* is already used by a bus link (**OPENBUS (0x05)**), the pin can not be used as a GPIO thus the end device will return error code **R_FORBIDDEN**
*pinMode* :
- Digital Input (**E_INPUT_D = 1**)
- Digital Output (**E_OUTPUT_D = 2**)
- Analog Input (**E_INPUT_A = 3**)
- PWM Output (**E_OUTPUT_A = 4**)
- Digital Input Pullup (**E_INPUT_PULLUP = 5**)
- Digital Trigger (**E_TRIGGER_D = 7**)
  - Will send a **READPIN (0x02)** packet on digital change then *pinMode* will be set on **E_NONE**
- Analog Trigger (**E_TRIGGER_A = 8**)
  - Will send a **READPIN (0x02)** packet on analog change then *pinMode* will be set on **E_NONE**
  - Expects *t_low* *t_high* and *inverted* to be set.
  - *inverted* == 1 means the end device will notify when the value on *numPin* is out of range [*t_low*,*t_high*]
- Infinite Digital Trigger (**E_INFINITE_TRIGGER_D = 9**)
  - Will send a **READPIN (0x02)** packet on every digital change
  - Expects a *inverted* boolean (example : `[0x65,0x09,0x00] -> Set Pin 5 as infinite trigger on rising edges`
  - example : `[0x68,0x09,0x01] -> Set Pin 8 as infinite trigger on falling edges`

example:
`[0x65,0x04] -> Set Pin 5 on PWM Output mode`


#### GETPINMODE (0x04)
`[0b10000000 + *numPin*] [] [] [] [] [] [] []`

Ask the end device to return the mode of a pin (*numPin*)
The end device will respond with a **GETPINMODE (0x04)** packet

example:\
`[0x81] -> get the mode Pin 1 is currently on`

#### OPENBUS (0x05)
`[0b10100000 + *numBus*] [*busSpeed* >> 16] [(*busSpeed* >> 8) & 0xFF] [*busSpeed* & 0xFF]`

Ask the end device to open the specified bus (*numBus*) at the specified speed (*busSpeed*)
*numBus*:
- Serial Bus 0 (*0x00*):
  - pins PA9 (Tx), PA10 (Rx)
- Serial Bus 1 (*0x01*):
  - pins PA2 (Tx), PA3 (Rx)
- Serial Bus 2 (*0x02*)
  - pins PB10 (Tx), PB11 (Rx)
- I2C Bus 0 (*0x03*):
  - pins PB6 (SCL), PB7 (SDA)
- I2C Bus 1 (*0x04*):
  - pins PB10 (SCL), PB11 (SDA)
Once the bus is opened, the end device will transmit **READBUS (0x07)** packets on data reception

example:\
`[0xA0,0x00,0x25,0x80] -> Open Serial Bus 0 at 9600 bauds`

#### CLOSEBUS (0x06)
`[0b11000000 + *numBus*] [*busSpeed* >> 16] [(*busSpeed* >> 8) & 0xFF] [*busSpeed* & 0xFF]`

Ask the end device to close the specified bus (*numBus*)
If the bus was already closed, the end device will respond with error code **R_FORBIDDEN**

example:\
`[0xC0] -> Close Serial Bus 0`

#### BUSCOMM (0x07)
`[0b11100000 + *numBus*] [*char0*] [*char1*] [*char2*] [*char3*] [*char4*] [*char5*] [*char6*]`

Serial Bus: *char0* to *char6* are optionnal\
I2C Bus: *char0* defines the transmission mode
*char0* =
- **I2C_NONE = 0** add the data to the buffer
- **I2C_BEGIN = 1** start a transmission, add the data to the buffer and wait for a **I2C_END** to send it
- **I2C_FULL = 2** start a transmission, add the data to the buffer and send it, then release the bus
- **I2C_END = 3** add the data to the buffer and send it, then release the bus
- **I2C_REQUEST = 4** send a request to send *char2* bytes of data from adress *char1*
- **I2C_READ_FROM = 5** send a request to send *char3* bytes of data from the register *char2* on adress *char1*
- **I2C_END_KEEP = 6** add the data to the buffer and send it, then keep the connection active
- **I2C_FULL_KEEP = 7** start a transmission, add the data to the buffer and send it, then keep the connection active
- **I2C_SCAN_ADDR = 8** probe the bus for a device at adress *char1*, send an **ERROR** packet containing (**I2C_RET_STATUS**+*numBus*+the return code it was given)
  - 0 : success (*device found*)
  - 1 : data too long to fit in transmit buffer (*should not happen here*)
  - 2 : received NACK on transmit of address (*no device on this adress*)
  - 3 : received NACK on transmit of data (*bus error*)
  - 4 : other error

examples:\
`[0xE0,0x12,0x23,0x34]-> Send 0x12,0x23,0x34 on the Serial Bus 0`\
`[0xE3,0x05,0x03,0xFF,0x02]-> Request 2 bytes of data from the register 0xFF of the device at adress 0x03 on the I2C Bus 0`
### End Device

#### ERROR (0x00)
`[0x00] [ERR_TYPE] [NUM_STM32]`

Transmit various control informations
ERR_TYPE:
- 0x00 = R_OK
- 0x10 = R_INVALID_FMT
- 0x20 = R_FORBIDDEN
- 0x30 = R_INVALID_PARAM
- 0x40 = I2C_RET_STATUS
- 0xFF = R_ALIVE

#### READBUS (0x07)
`[0b01110000 + *numBus*] [NUM_STM32] [*char0*] [*char1*] [*char2*] [*char3*] [*char4*] [*char5*]`

Serial Bus: *char0* to *char5* are optionnal

#### READPIN (0x02)
`[0b00100000 + *numPin*] [NUM_STM32] [*val* << 8] [*val* & 0xFF]`

#### GETPINMODE (0x03)
`[0b00110000 + *numPin*] [NUM_STM32] [*PIN_MODE*] [*T_low* << 8] [*T_low* & 0xFF] [*T_high* << 8] [*T_high* & 0xFF] [*inverted*]`

