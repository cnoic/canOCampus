# canOCampus
Connect to an array of sensors through a Can Bus




## Protocol
### Master
#### SETPIN (0x01)
`[0b00100000 + *numPin*] [*valPin* <<8] [*valPin* &0xFF] [] [] [] [] []`
Ask the end device to change the value (*valPin*) of a GPIO (*numPin*)
- The pin must be configured as an output
  - Else the end device will return error code **R_FORBIDDEN**
- *valPin* is on to bits to enable 10bit resolution on PWM if needed in future, for now the 2nd char can be set to 0x00 as the analogWriteResolution is on default (8 bit)

#### GETPIN (0x02)
`[0b01000000 + *numPin*] [] [] [] [] [] [] []`
Ask the end device to return the value of the GPIO (*numPin*)
- The pin must be configured as an Input
  - Else the end device will return error code **R_FORBIDDEN**
- The end device will respond with a **READPIN (0x02)** packet

#### SETPINMODE (0x03)
`[0b01100000 + *numPin*] [*pinMode*] [] [] [] [] [] []`
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


#### GETPINMODE (0x04)
`[0b10000000 + *numPin*] [] [] [] [] [] [] []`
Ask the end device to return the mode of a pin (*numPin*)



### End Device

