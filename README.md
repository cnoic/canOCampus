# canOCampus
Connect to an array of sensors through a Can Bus




## Protocol
### Master
#### SETPIN (0x01)
`[0b00100000 + *numPin*] [*valPin* >> 8] [*valPin* &0xFF]`

Ask the end device to change the value (*valPin*) of a GPIO (*numPin*)
- The pin must be configured as an output
  - Else the end device will return error code **R_FORBIDDEN**
- *valPin* is on 2 char to enable 10 bit resolution on PWM if needed in future, for now the 1st char can be set to 0x00 as the analogWriteResolution is on default (8 bit)

example :
`[0x21,0x00,0x5F] -> set Pin 1 to 95/255`

#### GETPIN (0x02)
`[0b01000000 + *numPin*]`

Ask the end device to return the value of the GPIO (*numPin*)
- The pin must be configured as an Input
  - Else the end device will return error code **R_FORBIDDEN**
- The end device will respond with a **READPIN (0x02)** packet

example :
`[0x41] -> Get the value of Pin 1`

#### SETPINMODE (0x03)
`[0b01100000 + *numPin*] [*pinMode*]`

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
- Infinite Digital Trigger (**E_INFINITE_TRIGGER_D = 9**)
  - Will send a **READPIN (0x02)** packet on every digital change

example:
`[0x65,0x04] -> Set Pin 5 on PWM Output mode`


#### GETPINMODE (0x04)
`[0b10000000 + *numPin*] [] [] [] [] [] [] []`

Ask the end device to return the mode of a pin (*numPin*)
The end device will respond with a **GETPINMODE (0x04)** packet

example:
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
example:
`[0xA0,0x00,0x25,0x80] -> Open Serial Bus 0 at 9600 bauds`

#### OPENBUS (0x05)
`[0b11000000 + *numBus*] [*busSpeed* >> 16] [(*busSpeed* >> 8) & 0xFF] [*busSpeed* & 0xFF]`

Ask the end device to close the specified bus (*numBus*)
If the bus was already closed, the end device will respond with error code **R_FORBIDDEN**
`[0xC0] -> Close Serial Bus 0`

#### BUSCOMM (0x07)
`[0b11100000 + *numBus*] [ [*char0*] [*char1*] [*char2*] [*char3*] [*char4*] [*char5*] [*char6] ]`
- Serial Bus:
  *char0* to *char6* are optionnal
- I2C Bus:
  *char0* defines the transmission mode 
    -**I2C_NONE = 0** add the data to the buffer
    -**I2C_BEGIN = 1** start a transmission, add the data to the buffer and wait for a **I2C_END** to send it
    -**I2C_FULL = 2** start a transmission, add the data to the buffer and send it, then release the bus
    -**I2C_END = 3** add the data to the buffer and send it, then release the bus
    -**I2C_REQUEST = 4** send a request to send *char2* bytes of data from adress *char1*
    -**I2C_READ_FROM = 5** send a request to send *char3* bytes of data from the register *char2* on adress *char1*
    -**I2C_END_KEEP = 6** add the data to the buffer and send it, then keep the connection active
    -**I2C_FULL_KEEP = 7** start a transmission, add the data to the buffer and send it, then keep the connection active
    -**I2C_SCAN_ADDR = 8** probe the bus for a device at adress *char1*, send an **ERROR** packet containing (**I2C_RET_STATUS**+*numBus*+the return code it was given)
      - 0:success (*device found*)
      - 1:data too long to fit in transmit buffer (*should not happen here*)
      - 2:received NACK on transmit of address (*no device on this adress*)
      - 3:received NACK on transmit of data (*bus error*)
      - 4:other error
    
### End Device

