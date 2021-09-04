# canOCampus
Connect to an array of sensors through a Can Bus




## Protocol
### SETPIN (0x01)
[__0b001 + *numPin*__] [__*valPin* <<8__] [__*valPin* &0xFF__]
