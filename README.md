# canOCampus
Connect to an array of sensors through a Can Bus




## Protocol
### SETPIN (0x01)
[0b00100000 + *numPin*] [*valPin* <<8] [*valPin* &0xFF]
