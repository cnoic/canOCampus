//sudo ip link set can0 up type can bitrate 50000

#include <EEPROM.h>
#include <Wire.h>


#define DEBUG 0
#define ECHOSERIAL 0
#define MONID 0x01 //id propre de chaque stm32
#define TALKBACK 1 //informer en cas d erreur
#define REP_ID 0x10 //id to respond
#define NBBUS 5

TwoWire Wire2(PB11, PB10);
HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial3(PB11, PB10);


typedef enum {R_OK = 0x00, R_INVALID_FMT = 0x10, R_FORBIDDEN = 0x20, R_INVALID_PARAM = 0x30, I2C_RET_STATUS = 0x40, R_ALIVE = 0xFF} COMMAND_STATUS;

typedef enum {SETPIN = 0x01, GETPIN = 0x02, SETPINMODE = 0x03, GETPINMODE = 0x04, OPENBUS = 0x05, CLOSEBUS = 0x06, BUSCOMM = 0x07, C_ERROR = 0x00} CAN_COMMAND;

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_5KBPS_T1, CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;

typedef enum {B_SERIAL = 0, B_I2C = 1} M_BUS;
typedef enum {I2C_NONE = 0, I2C_BEGIN = 1, I2C_FULL = 2, I2C_END = 3, I2C_REQUEST = 4, I2C_READ_FROM = 5, I2C_END_KEEP = 6, I2C_FULL_KEEP = 7, I2C_SCAN_ADDR = 8} I2C_MODES;


typedef struct
{
  uint32_t id;        /* 29 bit identifier                               */
  uint8_t  data[8];   /* Data field                                      */
  uint8_t  len;       /* Length of data field in bytes                   */
  uint8_t  ch;        /* Object channel(Not use)                         */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct
{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;

typedef struct
{
  uint8_t num;
  bool analog;
  bool trigger;
  bool infinite;
  bool reserved;
  bool active;
  bool in;
  bool pullup;
  bool pulldown;
  bool pwm;
  uint16_t val;
  uint16_t t_low;
  uint16_t t_high;
  bool inverted;
} Pin;

typedef struct
{
  uint8_t num;
  bool enabled;
  int pin1;
  int pin2;
  M_BUS type;
  uint8_t *buf;
  uint8_t len_buf;
  long int spd;
} Bus;


CAN_bit_timing_config_t can_configs[8] = {{2,13,100}, {2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};

const int p_act[29] = {PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PA8,PA9,PA10,PA11,PA12,PA15,PB0,PB1,PB3,PB4,PB5,PB6,PB7,PB10,PB11,PB13,PB14,PB15,PC13,PC14,PC15};
const int p_pwm[15] = {PA0, PA1, PA2, PA3, PA6, PA7, PA8, PA9, PA10, PB0, PB1, PB6, PB7, PB8, PB9};

Pin * default_pins(){
  Pin *out = (Pin *) malloc(29*sizeof(Pin));
  for(int p = 0; p < 29; p++){
    out[p].num = p_act[p];
    out[p].active = false;
    out[p].reserved = false;
    out[p].trigger = false;
    out[p].analog = false;
    out[p].infinite = false;
    out[p].pullup = false;
    out[p].pulldown = false;
    out[p].in = false;
    for(int l =0; l<15; l++){
      if(p_act[p] == p_pwm[l]) out[p].pwm = true;
    }
    out[p].val = 0;
    out[p].t_low = 0;
    out[p].t_high = 0;
    out[p].inverted = false;
  }
  return out;
}


Bus * default_buses(){
  Bus *out = (Bus *) malloc(NBBUS*sizeof(Bus));
  out[0].num = 0;
  out[0].enabled = false;
  out[0].pin1 = 9;
  out[0].pin2 = 10;
  out[0].type = B_SERIAL ;
  out[0].buf = (uint8_t *) malloc(7*sizeof(uint8_t));
  out[0].len_buf = 0;
  out[0].spd = 9600;

  out[1].num = 1;
  out[1].enabled = false;
  out[1].pin1 = 2;
  out[1].pin2 = 3;
  out[1].type = B_SERIAL ;
  out[1].buf = (uint8_t *) malloc(7*sizeof(uint8_t));
  out[1].len_buf = 0;
  out[1].spd = 9600;

  out[2].num = 2;
  out[2].enabled = false;
  out[2].pin1 = 21;
  out[2].pin2 = 22;
  out[2].type = B_SERIAL ;
  out[2].buf = (uint8_t *) malloc(7*sizeof(uint8_t));
  out[2].len_buf = 0;
  out[2].spd = 9600;

  out[3].num = 3;
  out[3].enabled = false;
  out[3].pin1 = 19;
  out[3].pin2 = 20;
  out[2].type = B_I2C ;
  out[3].buf = (uint8_t *) malloc(7*sizeof(uint8_t));
  out[3].len_buf = 0;
  out[3].spd = 400000UL;

  out[4].num = 4;
  out[4].enabled = false;
  out[4].pin1 = 21;
  out[4].pin2 = 22;
  out[4].type = B_I2C ;
  out[4].buf = (uint8_t *) malloc(7*sizeof(uint8_t));
  out[4].len_buf = 0;
  out[4].spd = 400000UL;
  
  return out;
}



/**
 * global vars
 */
Pin* p;
Bus* b;




/**
 * Print registers.
*/ 
void printRegister(char * buf, uint32_t reg) {
  if (DEBUG == 0) return;
  Serial.print(buf);
  Serial.print(reg, HEX);
  Serial.println();
}

void reset_pins(){
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13,LOW);
  free(p);
  p = default_pins();
  savePins(p);
  digitalWrite(PC13,HIGH);
}


/**é
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */

 
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 27) return;

  CAN1->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN1->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN1->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
    if (mode == 0) {
    CAN1->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN1->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN1->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN1->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN1->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN1->FA1R |= (0x1UL<<index);                // Activate filter

}

/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 * @params: remap   - Select CAN port. 
 *                    =0:CAN_RX mapped to PA11, CAN_TX mapped to PA12
 *                    =1:Not used
 *                    =2:CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)
 *                    =3:CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)
 *
 */
bool CANInit(BITRATE bitrate, int remap)
{
  // Reference manual
  // https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf

  RCC->APB1ENR |= 0x2000000UL;       // Enable CAN clock 
  RCC->APB2ENR |= 0x1UL;             // Enable AFIO clock
  AFIO->MAPR   &= 0xFFFF9FFF;        // reset CAN remap
                                     // CAN_RX mapped to PA11, CAN_TX mapped to PA12

  if (remap == 0) {
    RCC->APB2ENR |= 0x4UL;           // Enable GPIOA clock
    GPIOA->CRH   &= ~(0xFF000UL);    // Configure PA12(0b0000) and PA11(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOA->CRH   |= 0xB8FFFUL;       // Configure PA12(0b1011) and PA11(0b1000)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     
    GPIOA->ODR |= 0x1UL << 12;       // PA12 Upll-up
    
  }
                                
  if (remap == 2) {
    AFIO->MAPR   |= 0x00004000;      // set CAN remap
                                     // CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)

    RCC->APB2ENR |= 0x8UL;           // Enable GPIOB clock
    GPIOB->CRH   &= ~(0xFFUL);       // Configure PB9(0b0000) and PB8(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOB->CRH   |= 0xB8UL;          // Configure PB9(0b1011) and PB8(0b1000)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     
    GPIOB->ODR |= 0x1UL << 8;        // PB8 Upll-up
  }
  
  if (remap == 3) {
    AFIO->MAPR   |= 0x00005000;      // set CAN remap
                                     // CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)

    RCC->APB2ENR |= 0x20UL;          // Enable GPIOD clock
    GPIOD->CRL   &= ~(0xFFUL);       // Configure PD1(0b0000) and PD0(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOD->CRH   |= 0xB8UL;          // Configure PD1(0b1011) and PD0(0b1000)
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     
    GPIOD->ODR |= 0x1UL << 0;        // PD0 Upll-up
  }

  CAN1->MCR |= 0x1UL;                   // Require CAN1 to Initialization mode 
  while (!(CAN1->MSR & 0x1UL));         // Wait for Initialization mode

  //CAN1->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
  CAN1->MCR = 0x41UL;                   // Hardware initialization(With automatic retransmission)
   
  // Set bit rates 
  CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
  CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);

  // Configure Filters to default values
  CAN1->FMR  |=   0x1UL;                // Set to filter initialization mode
  CAN1->FMR  &= 0xFFFFC0FF;             // Clear CAN2 start bank

  // bxCAN has 28 filters.
  // These filters are used for both CAN1 and CAN2.
  // STM32F103 has only CAN1, so all 28 are used for CAN1
  CAN1->FMR  |= 0x1C << 8;              // Assign all filters to CAN1

  // Set fileter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL); 

  //CANSetFilter(0, 0, 1, 0, 0x0UL, (unsigned long) 0xFF + MONID); 
  
  CAN1->FMR   &= ~(0x1UL);              // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;
  bool can1 = false;
  CAN1->MCR   &= ~(0x1UL);              // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN1->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //Serial.print("can1=");
  //Serial.println(can1);
  if (!can1) return false;
  return true; 
}


#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU
 
/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
void CANReceive(CAN_msg_t* CAN_rx_msg)
{
  uint32_t id = CAN1->sFIFOMailBox[0].RIR;
  if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
      CAN_rx_msg->format = STANDARD_FORMAT;;
      CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
  } 
  else {                               // Extended frame format
      CAN_rx_msg->format = EXTENDED_FORMAT;;
      CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
  }

  if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
      CAN_rx_msg->type = DATA_FRAME;
  }
  else {                               // Remote frame
      CAN_rx_msg->type = REMOTE_FRAME;
  }

  
  CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
  
  CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
  CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
  CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  // Release FIFO 0 output mailbox.
  // Make the next incoming message available.
  CAN1->RF0R |= 0x20UL;
}
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @params CAN_tx_msg - CAN message structure for transmission
 * 
 */
void CANSend(CAN_msg_t* CAN_tx_msg)
{
  volatile int count = 0;

  uint32_t out = 0;
  if (CAN_tx_msg->format == EXTENDED_FORMAT) { // Extended frame format
      out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  }
  else {                                       // Standard frame format
      out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
  }

  // Remote frame
  if (CAN_tx_msg->type == REMOTE_FRAME) {
      out |= STM32_CAN_TIR_RTR;
  }

  CAN1->sTxMailBox[0].TDTR &= ~(0xF);
  CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
  
  CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                               ((uint32_t) CAN_tx_msg->data[2] << 16) |
                               ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[0]      ));
  CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                               ((uint32_t) CAN_tx_msg->data[6] << 16) |
                               ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[4]      ));

  // Send Go
  CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

  // Wait until the mailbox is empty
  while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);

  // The mailbox don't becomes empty while loop
  /*if (CAN1->sTxMailBox[0].TIR & 0x1UL) {
    Serial.println("Send Fail");
    Serial.println(CAN1->ESR);
    Serial.println(CAN1->MSR);
    Serial.println(CAN1->TSR);
  }*/
}

 /**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
uint8_t CANMsgAvail(void)
{
  // Check for pending FIFO 0 messages
  return CAN1->RF0R & 0x3UL;
}


/**
 *  Update the pin to make the values sored correspond w/ the physical io state
 */

bool updatePin(Pin *p){
  if(! p->active) return false;
  if(p->in){
    if(p->analog) {p->val = analogRead(p->num);}
    else {p->val = digitalRead(p->num);}
  }else{
    if(p->analog) {analogWrite(p->num, p->val);}
    else {digitalWrite(p->num, (p->val ? HIGH : LOW));}
  }
  return true;
}

bool process_i2c(TwoWire *w, Bus *b){
  if(b->len_buf) {
    uint8_t i = 1;
    if((b->buf[0] == I2C_REQUEST) && (b->len_buf > 2)){
      w->requestFrom(b->buf[1], b->buf[2]);
    }else{
      if ((b->buf[0] == I2C_READ_FROM) && (b->len_buf >= 2)){
          w->beginTransmission(b->buf[1]);
          w->write(b->buf[2]);
          confirm((COMMAND_STATUS) (w->endTransmission(false)));
          w->requestFrom(b->buf[1], b->buf[3]);
      }else if ((b->buf[0] == I2C_SCAN_ADDR) && (b->len_buf == 2)){
          w->beginTransmission(b->buf[1]);
          confirm((COMMAND_STATUS) (w->endTransmission() + I2C_RET_STATUS + (b->num  << 3)));
      }else{
        if ((b->buf[0] == I2C_BEGIN || b->buf[0] == I2C_FULL) && (b->len_buf >= 2)){
            w->beginTransmission(b->buf[1]);
            i = 2;
        }
        if(b->len_buf > i) w->write(&(b->buf[i]), b->len_buf-i);
        if (b->buf[0] == I2C_END || b->buf[0] == I2C_FULL){
            //w->endTransmission();
            confirm((COMMAND_STATUS) (w->endTransmission()));
        }
        if (b->buf[0] == I2C_END_KEEP || b->buf[0] == I2C_FULL_KEEP){
            //w->endTransmission();
            confirm((COMMAND_STATUS) (w->endTransmission(false)));
        }
      }
    }
    b->len_buf = 0;
  }
  if (w->available()){
    while(w->available() && b->len_buf < 6){
       b->buf[b->len_buf] = w->read();
       b->len_buf ++;
       delayMicroseconds((1000000 / b->spd)*16);
    }
    return true;
  }
  return false;
}

bool process_serial(HardwareSerial *s, Bus *b){
  if(b->len_buf){
    s->write(b->buf, b->len_buf);
    b->len_buf = 0;
  }
  if (s->available()){
    while(s->available() && b->len_buf < 6){
       b->buf[b->len_buf] = s->read();
       b->len_buf ++;
       if(!s->available()) delay(2);
    }
    return true;
  }
  return false;
}


bool updateBus(Bus *b){ // renvoie true si quelque chose a été lu
  if (!b->enabled) return false;
  if(b->num == 0) return (process_serial(&Serial1,b));
  if(b->num == 1) return (process_serial(&Serial2,b));
  if(b->num == 2) return (process_serial(&Serial3,b));
  if(b->num == 3) return (process_i2c(&Wire,b));
  if(b->num == 4) return (process_i2c(&Wire2,b));
  return false;
}

bool updatePinMode(Pin *p){
  if(p->reserved) return false;
  if(p->in){
    if(p->pullup && p-> pulldown) return false;
    if(p->pullup) {pinMode(p->num,INPUT_PULLUP);}
    else if(p->pulldown) {pinMode(p->num, INPUT_PULLDOWN);}
    else {pinMode(p->num,INPUT);}
  }else{
    if (p->analog && (! p->pwm)) return false;
    pinMode(p->num,OUTPUT);
  }
  p->active = true;
  return true;
}

bool openBus(Pin * pins, Bus *b){
  if(! (b->enabled)){
    if(pins[b->pin1].reserved || pins[b->pin2].reserved) return false;
    pins[b->pin1].reserved = true;
    pins[b->pin1].active = false;
    pins[b->pin2].reserved = true;
    pins[b->pin2].active = false;
    switch (b->num) {
      case 0:
        Serial1.begin(b->spd);
      break;
      case 1:
        Serial2.begin(b->spd);
      break;
      case 2:
        Serial3.begin(b->spd);
      break;
      case 3:
        Wire.begin();
        Wire.setClock(b->spd);
      break;
      case 4:
        Wire2.begin();
        Wire2.setClock(b->spd);
      break;
      default:
        return false;
    }
    b->enabled = true;
    return true;
  }
  return false;
}

void closeBus(Pin * pins, Bus *b){
  pins[b->pin1].reserved = false;
  pins[b->pin2].reserved = false;
  if(!b->enabled) return;
  if(b->num == 0) Serial1.end();
  if(b->num == 1) Serial2.end();
  if(b->num == 2) Serial3.end();
  if(b->num == 3) Wire.end();
  if(b->num == 4) Wire2.end();
  b->enabled = false;
}


void savePins(Pin* p){
  uint8_t id = MONID;
  EEPROM.put(0,id);
  for(int i =0; i< 29; i++){
    EEPROM.put(i*sizeof(Pin) + sizeof(uint8_t), p[i]);
  }
}


bool restorePins(Pin* p){
  uint8_t id;
  EEPROM.get(0,id);
  if (id != (uint8_t) MONID) return false;
  for(int i =0; i< 29; i++){
    EEPROM.get(i*sizeof(Pin) + sizeof(uint8_t), p[i]);
    if(p[i].active) updatePinMode(&(p[i]));
  }
  return true;
}


COMMAND_STATUS CANSetPinMode(Pin* p, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len < 2) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > 28) return R_INVALID_PARAM;
  p[num].in =  CAN_rx_msg->data[1]& 0x80;
  p[num].analog = CAN_rx_msg->data[1]& 0x40;
  p[num].pullup = CAN_rx_msg->data[1]& 0x20;
  p[num].pulldown = CAN_rx_msg->data[1]& 0x10;
  p[num].trigger = CAN_rx_msg->data[1]& 0x08;
  p[num].infinite = CAN_rx_msg->data[1]& 0x04;
  p[num].inverted = CAN_rx_msg->data[1]& 0x02;
  if (CAN_rx_msg->len == 6){
  p[num].t_low = (CAN_rx_msg->data[2]) << 8;
  p[num].t_low += (CAN_rx_msg->data[3]);
  p[num].t_high = (CAN_rx_msg->data[4]) << 8;
  p[num].t_high += (CAN_rx_msg->data[5]);
  }
  if (!updatePinMode(&(p[num]))) return R_FORBIDDEN;
  savePins(p);
  return R_OK;
}


COMMAND_STATUS CANOpenBus(Pin* p, Bus* b, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 4) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > NBBUS) return R_INVALID_PARAM;
  if (b[num].enabled) return R_FORBIDDEN;
  b[num].spd = CAN_rx_msg->data[1] << 0x10;
  b[num].spd += CAN_rx_msg->data[2] << 0x08;
  b[num].spd += CAN_rx_msg->data[3];
  //if(b[num].spd > 0x38400) return R_FORBIDDEN;
  if (!openBus(p, &(b[num]))) return R_FORBIDDEN;
  return R_OK;
}


COMMAND_STATUS CANCloseBus(Pin* p, Bus* b, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 1) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > NBBUS) return R_INVALID_PARAM;
  if (! b[num].enabled) return R_FORBIDDEN;
  b[num].enabled = false;
  closeBus(p, &(b[num]));
  return R_OK;
}

COMMAND_STATUS CANGetPinMode(Pin* p, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 1) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > 28) return R_INVALID_PARAM;
  CAN_msg_t CAN_TX_msg;
  CAN_TX_msg.data[0] = (GETPINMODE << 5) + num;
  CAN_TX_msg.data[1] = MONID;
  CAN_rx_msg->data[2] = p[num].in << 7;
  CAN_rx_msg->data[2] |= p[num].analog << 6;
  CAN_rx_msg->data[2] |= p[num].pullup << 5;
  CAN_rx_msg->data[2] |= p[num].pulldown << 4;
  CAN_rx_msg->data[2] |= p[num].trigger << 3;
  CAN_rx_msg->data[2] |= p[num].infinite << 2;
  CAN_rx_msg->data[2] |= p[num].inverted << 1;
  CAN_TX_msg.len = 3;
  CAN_TX_msg.type = DATA_FRAME;
  CAN_TX_msg.format = STANDARD_FORMAT;
  CAN_TX_msg.id = REP_ID;
  CANSend(&CAN_TX_msg);
  return R_OK;
}


COMMAND_STATUS CANGetPinValue(Pin* p, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 1) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > 28) return R_INVALID_PARAM;
  if(p[num].reserved || (!p[num].active)) return R_FORBIDDEN;
  updatePin(&(p[num]));
  return CANSendPinValue(p,num);
}

COMMAND_STATUS CANSetPinValue(Pin* p, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 3) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > 28) return R_INVALID_PARAM;
  if(p[num].in) return R_FORBIDDEN;
  p[num].val = (CAN_rx_msg->data[1]) << 8;
  p[num].val += (CAN_rx_msg->data[2]);
  updatePin(&(p[num]));
  return R_OK;
}

COMMAND_STATUS CANWriteBus(Bus* b, CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len > 8) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[0] & 0x1F;
  if(num > NBBUS) return R_INVALID_PARAM;
  if(! b[num].enabled) return R_FORBIDDEN;
  for(int i = 1; i < CAN_rx_msg->len; i++){
    b[num].buf[i-1] = CAN_rx_msg->data[i];
  }
  b[num].len_buf = CAN_rx_msg->len - 1;
  return R_OK;
}


COMMAND_STATUS CANReadBus(Bus* b){
  CAN_msg_t CAN_TX_msg;
  CAN_TX_msg.data[0] = (BUSCOMM << 5) + b->num;
  CAN_TX_msg.data[1] = MONID;
  for(int i = 0; i < b->len_buf; i++){
    CAN_TX_msg.data[2+i] = b->buf[i];
  }
  CAN_TX_msg.len = 2+ (b->len_buf);
  if (! ECHOSERIAL) b->len_buf = 0;
  CAN_TX_msg.type = DATA_FRAME;
  CAN_TX_msg.format = STANDARD_FORMAT;
  CAN_TX_msg.id = REP_ID;
  CANSend(&CAN_TX_msg);
  return R_OK;
}

COMMAND_STATUS CANSendPinValue(Pin* p, uint16_t num){
  CAN_msg_t CAN_TX_msg;
  CAN_TX_msg.data[0] = (GETPIN << 5) + num;
  CAN_TX_msg.data[1] = MONID;
  CAN_TX_msg.data[2] = p[num].val >> 8;
  CAN_TX_msg.data[3] = p[num].val && 0xFF;
  CAN_TX_msg.len = 4;
  
  CAN_TX_msg.type = DATA_FRAME;
  CAN_TX_msg.format = STANDARD_FORMAT;
  CAN_TX_msg.id = REP_ID;
  CANSend(&CAN_TX_msg);
  return R_OK;
}

COMMAND_STATUS CANReset(CAN_msg_t* CAN_rx_msg){
  if(CAN_rx_msg->len != 2) return R_INVALID_FMT;
  uint8_t num = CAN_rx_msg->data[1];
  if(((~num) & 0xFF) != MONID) return R_INVALID_PARAM;
  reset_pins();
  return R_OK;
}



void confirm(COMMAND_STATUS c){
  if(TALKBACK && c != R_OK){
     CAN_msg_t CAN_TX_msg;
     CAN_TX_msg.data[0] = (C_ERROR << 5);
     CAN_TX_msg.data[1] = MONID;
     CAN_TX_msg.data[2] = (uint8_t) c;
     CAN_TX_msg.len = 3;
     
     CAN_TX_msg.type = DATA_FRAME;
     CAN_TX_msg.format = STANDARD_FORMAT;
     CAN_TX_msg.id = REP_ID;
     CANSend(&CAN_TX_msg);
  }
}


void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13,LOW);
  //Serial.begin(9600);
  delay(1000);
  //bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  bool ret = CANInit(CAN_50KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  //bool ret = CANInit(CAN_500KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  p = default_pins();
  b = default_buses();
  bool rest = false;
  rest = restorePins(p); //comment to reset conf each startup
  if (!rest){ //conf restore failed
    for(int i =0; i< 5; i++){
    digitalWrite(PC13,HIGH);
    delay(100);
    digitalWrite(PC13,LOW);
    delay(150);
    }
    digitalWrite(PC13,LOW);
    delay(500);
    reset_pins();
  }
  if (!ret){ //can startup failed
    while(true){
      digitalWrite(PC13,LOW);
      delay(800);
      digitalWrite(PC13,HIGH);
      delay(150);
      digitalWrite(PC13,LOW);
      delay(100);
      digitalWrite(PC13,HIGH);
      delay(150);
      digitalWrite(PC13,LOW);
      delay(100);
      digitalWrite(PC13,HIGH);
      delay(1000);
    }
  }
  digitalWrite(PC13,HIGH);
}

CAN_msg_t CAN_RX_msg;
void loop() {
  
  for(int i =0; i<5; i++){
    if(updateBus(&(b[i]))){
      confirm(CANReadBus(&(b[i])));
    }
  }
  for(int i =0; i<29; i++){
    if(p[i].trigger && p[i].in){
      updatePin(&(p[i]));
      if(((p[i].analog) && (((p[i].t_low <= p[i].val) && (p[i].t_high > p[i].val)) != p[i].inverted)) ||
      ((!p[i].analog) && ((bool) p[i].val) != p[i].inverted))
      {
          if(p[i].infinite){ p[i].inverted = !p[i].inverted;}
          else{ p[i].trigger = false;}
          confirm(CANSendPinValue(p,i));
          savePins(p);
        }
      }
  }
  if(CANMsgAvail()) {
    CANReceive(&CAN_RX_msg);
    if(CAN_RX_msg.id == MONID+0xFF){ //direct command
      if(CAN_RX_msg.type == DATA_FRAME){// command
        if(CAN_RX_msg.len == 0) return; // ignore empty messages
        switch ((CAN_COMMAND) (CAN_RX_msg.data[0] >> 5) ){
          case SETPIN:
              confirm(CANSetPinValue(p, &CAN_RX_msg));
          break;
          case GETPIN:
              confirm(CANGetPinValue(p, &CAN_RX_msg));
          break;
          case SETPINMODE:
              confirm(CANSetPinMode(p, &CAN_RX_msg));
          break;
          case GETPINMODE:
              confirm(CANGetPinMode(p, &CAN_RX_msg));
          break;
          case OPENBUS:
              confirm(CANOpenBus(p, b, &CAN_RX_msg));
          break;
          case CLOSEBUS:
              confirm(CANCloseBus(p, b, &CAN_RX_msg));
          break;
          case BUSCOMM:
              confirm(CANWriteBus(b, &CAN_RX_msg));
          break;
          case C_ERROR:
              confirm(CANReset(&CAN_RX_msg));
          break;
          default:
          break;
        }
      }
    }
  }
}

/* TODO
 *  Système de detection (scan des id)
 *  BUS SPI
 *  TESTER / VALIDER
 *  
 */
