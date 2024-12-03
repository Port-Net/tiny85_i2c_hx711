#include <Arduino.h>

/**
 * Example sketch for writing to and reading from a slave in transactional manner
 *
 * NOTE: You must not use delay() or I2C communications will fail, use tws_delay() instead (or preferably some smarter timing system)
 *
 * On write the first byte received is considered the register addres to modify/read
 * On each byte sent or read the register address is incremented (and it will loop back to 0)
 *
 * You can try this with the Arduino I2C REPL sketch at https://github.com/rambo/I2C/blob/master/examples/i2crepl/i2crepl.ino 
 * If you have bus-pirate remember that the older revisions do not like the slave streching the clock, this leads to all sorts of weird behaviour
 *
 * To read third value (register number 2 since counting starts at 0) send "[ 8 2 [ 9 r ]", value read should be 0xBE
 * If you then send "[ 9 r r r ]" you should get 0xEF 0xDE 0xAD as response (demonstrating the register counter looping back to zero)
 *
 * You need to have at least 8MHz clock on the ATTiny for this to work (and in fact I have so far tested it only on ATTiny85 @8MHz using internal oscillator)
 * Remember to "Burn bootloader" to make sure your chip is in correct mode 
 */

/*
 * Config register 0x00, len 2: 
 * Byte 1:
 *  | reboot | write EE | Rate | cont read | Channel B mode | Channel A mode |
 *   Channel A Mode (Bit 0, 1): 0 -> off, 1 -> gain 128, 2 -> gain 32
 *   Channel B Mode (Bit 2): 0 -> off, 1 -> gain 32
 *   Cont Read (Bit 3, 4): 0 -> read reg, 1 -> scale1 + scale 2, 2 -> continue over all
 *   Rate (Bit 5): 0 -> 10SPS, 1 -> 80SPS (not connected)
 *   Write EE (Bit 6): 1 -> write config, tara, i2c address to EEPROM
 *   Reboot (Bit 7): 1 -> reboot
 * Byte 2:
 *   AVG Count: exponential averaging counter
 * other registers:
 * name        | addr | len
 * --------------------------
 * i2c_addr    | 0x01 | 1
 * tara A      | 0x02 | 3
 * tara B      | 0x03 | 3
 * scale A     | 0x04 | 3
 * scale B     | 0x05 | 3
 * avg scale A | 0x06 | 3
 * avg scale B | 0x07 | 3
 * raw scale A | 0x08 | 3
 * raw scale B | 0x09 | 3
 * status      | 0x0a | 1
 */

#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include "Adafruit_HX711.h"

#define DEFAULT_I2C_SLAVE_ADDRESS 0x34 // the 7-bit address (remember to change this when adapting this example)

#define HX711_DT  4
#define HX711_SCK 3
#define HX711_RATE_PIN 1

#define NO_CHANNEL -1
#define CHANNEL_A 0
#define CHANNEL_B 1

#define CHANNEL_A_MASK     0x03
#define CHANNEL_A_OFF      0x00
#define CHANNEL_A_GAIN_128 0x01
#define CHANNEL_A_GAIN_64  0x02
#define CHANNEL_B_MASK     0x04
#define CHANNEL_B_OFF      0x00
#define CHANNEL_B_GAIN_32  0x04
#define CONT_READ_MASK  0x18 
#define CONT_READ_SCALE 0x08 
#define CONT_READ_ALL   0x10 

#define RATE_FLAG         5
#define WRITE_EEPROM_FLAG 6
#define REBOOT_FLAG       7

#define STATUS_READY_FLAG    0
#define STATUS_PD_FLAG       1
#define STATUS_CHA_FLAG      2
#define STATUS_RUN_FLAG      3
#define STATUS_EEPROM_FLAG   6
#define STATUS_REBOOT_FLAG   7

Adafruit_HX711 hx711(HX711_DT, HX711_SCK);

// for allowed buffer sizes see usiTwiSlave.h
//#define TWI_RX_BUFFER_SIZE ( 16 ) 
// Get this from https://github.com/rambo/TinyWire
#include "TinyWireS.h"

// Tracks the current register
volatile uint8_t current_reg = 0;
// Tracks the current register
volatile uint8_t reg_pos = 0;
// Tracks the current register sub pointer position
volatile uint8_t reg_sub_pos = 0;

enum Register_num_t {
  config_reg_num = 0,
  i2c_addr_reg_num = 1,
  tara1_reg_num = 2,
  tara2_reg_num = 3,
  scale1_reg_num = 4,
  scale2_reg_num = 5,
  avg_scale1_reg_num = 6,
  avg_scale2_reg_num = 7,
  raw_scale1_reg_num = 8,
  raw_scale2_reg_num = 9,
  status_reg_num = 10,
  num_regs = 11
};
uint8_t reg_len[num_regs] = {2,1,3,3,3,3,3,3,3,3,1};

#pragma pack(push, 1)
struct three_bytes_t {
  uint8_t b0;
  uint8_t b1;
  uint8_t b2;
};

union regs_t {
  uint8_t raw[27];
  uint8_t conf[9];
  struct content_t {
    uint16_t config_reg;
    uint8_t i2c_addr;
    three_bytes_t tara[2];
    three_bytes_t scale[2];
    three_bytes_t avg_scale[2];
    three_bytes_t raw_scale[2];
    uint8_t status_reg;
  } reg;
} regs;
#pragma pack(pop)

#define EE_MAGIC 42

volatile bool powerdown = false;
volatile bool reparse_config = true;
hx711_chanGain_t last_measured;

int32_t avg_scale_256[2];
uint8_t avg_ct[2];
uint8_t num_avg;
bool eeprom_config = false;
//uint8_t cmd_response[5];
//uint8_t cmd_response_len;
//uint8_t* cmd_response_p;
uint8_t timonel_buf[12] = {0x7D, APP_MAGIC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent() {
  // Timonel special
  if(current_reg == 0x82) {
    TinyWireS.send(timonel_buf[reg_sub_pos]);
    //TinyWireS.send('Q');
    reg_sub_pos++;
    if(reg_sub_pos >= 12) {
      reg_sub_pos = 0;
      current_reg = 0;
    }
    return;
  } else if(current_reg == 0x80) { // reset
    wdt_enable(WDTO_60MS);
  } else if(current_reg == 0x86) { // jump to bootloader
    ((void (*)(void))0)();
  }
  // std register access
  TinyWireS.send(regs.raw[reg_pos + reg_sub_pos]);
  // Increment the reg position on each read, and loop back to zero
  reg_sub_pos++;
  if (reg_sub_pos > reg_len[current_reg]) {
    reg_sub_pos = 0;
    switch(regs.reg.config_reg & CONT_READ_MASK) {
      case CONT_READ_SCALE:
        if((current_reg == scale1_reg_num) || (current_reg == avg_scale1_reg_num)) {
          reg_pos += reg_len[current_reg];
          current_reg++;
        } else if((current_reg == scale2_reg_num) || (current_reg == avg_scale2_reg_num)) {
          current_reg--;
          reg_pos -= reg_len[current_reg];
        }
        break;
      case CONT_READ_ALL:
        reg_pos += reg_len[current_reg];
        current_reg++;
        if(reg_pos > sizeof(regs.raw)) {
          reg_pos = 0;
          current_reg = 0;
        }
        break;
    }
  }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t count) {
  // some sanity checks
  if (count < 1) {
    return;
  }
  current_reg = TinyWireS.receive();
  TinyWireS.flushTxBuffer();
  reg_pos = 0;
  for(int i = 0; i < current_reg; ++i) {
    reg_pos += reg_len[i];
  }
  reg_sub_pos = 0;
  count--;
  if (!count) {
    // Timonel special
    if(current_reg == 0x80) { // RESETMCU
      TinyWireS.send(0x7F); // RESETACK
    } else if(current_reg == 0x82) { // GETTMNLV Command Get our Version
      TinyWireS.send(0x7D); // ACKTMNLV
      TinyWireS.send(APP_MAGIC); // in our case H instead of T
    } else if(current_reg == 0x86) { // EXITTMNL in our case we enter the bootloader
      TinyWireS.send(0x79); // ACKEXITT
    }
    // end Timonel special
    // This write was only to set the buffer for next read
    return;
  }
  for(int i = 0; i < min(count, reg_len[current_reg]); ++i) {
    regs.raw[reg_pos + i] = TinyWireS.receive();
  }
  reparse_config = true;
}

int32_t from_three_bytes(three_bytes_t scale) {
  int32_t value;
  value = scale.b2;
  value <<= 8;
  value |= scale.b1;
  value <<= 8;
  value |= scale.b0;
  if(scale.b2 & 0x80) {
    value |= 0xFF000000;
  }
  return value;
}

three_bytes_t to_three_bytes(int32_t i) {
  three_bytes_t ret;
  ret.b0 = i & 0xFF;
  ret.b1 = (i >> 8) & 0xFF;
  ret.b2 = (i >> 16) & 0xFF;
  return ret;
}

hx711_chanGain_t get_to_measure(hx711_chanGain_t last_measured) {
  hx711_chanGain_t next_measured;
  if(last_measured == CHAN_B_GAIN_32) {
    switch(regs.reg.config_reg & CHANNEL_A_MASK) {
      case CHANNEL_A_GAIN_128:
        next_measured = CHAN_A_GAIN_128;
        break;
      case CHANNEL_A_GAIN_64:
        next_measured = CHAN_A_GAIN_64;
        break;
      default:
        next_measured = last_measured;
    }
  } else {
    switch(regs.reg.config_reg & CHANNEL_B_MASK) {
      case CHANNEL_B_GAIN_32:
        next_measured = CHAN_B_GAIN_32;
        break;
      default:
        next_measured = last_measured;
    }
  }
  return next_measured;
}

void parse_config() {
  bool reboot = regs.reg.config_reg & (1<<REBOOT_FLAG);
  regs.reg.i2c_addr &= 0x7F;
  if(regs.reg.config_reg & (1<<WRITE_EEPROM_FLAG)) {
    EEPROM.update(1, EE_MAGIC);
    regs.reg.config_reg &= ~(1<<WRITE_EEPROM_FLAG);
    regs.reg.config_reg &= ~(1<<REBOOT_FLAG);
    EEPROM.put(2, regs.conf);
  }
  if(reboot) {
    wdt_enable(WDTO_60MS);
    while(true);
  }
#ifdef HAS_RATE_PIN
  if(regs.reg.config_reg & (1<<RATE_FLAG)) {
    digitalWrite(HX711_RATE_PIN, HIGH);
  } else {
    digitalWrite(HX711_RATE_PIN, LOW);
  }
#endif
  if(regs.reg.config_reg & (CHANNEL_A_MASK | CHANNEL_B_MASK)) {
    if(powerdown) {
      hx711.powerDown(false);
      delay(120);
      last_measured = get_to_measure(CHAN_B_GAIN_32);
      if(!hx711.isBusy()) {
        hx711.readChannelRaw(last_measured);
      }
      avg_ct[CHANNEL_A] = 0;
      avg_ct[CHANNEL_B] = 0;
      powerdown = false;
    }
    regs.reg.status_reg &= ~(1 << STATUS_PD_FLAG);
  } else {
    if(!powerdown) {
      hx711.powerDown(true);
      powerdown = true;
      regs.reg.status_reg &= ~(1<<STATUS_READY_FLAG);
    }
    regs.reg.status_reg |= (1<<STATUS_PD_FLAG);
  }
  num_avg = regs.reg.config_reg >> 8;
  if(eeprom_config) {
    regs.reg.status_reg |= (1<<STATUS_EEPROM_FLAG);
  } else {
    regs.reg.status_reg &= ~(1<<STATUS_EEPROM_FLAG);
  }
}

int8_t measure() {
  if(hx711.isBusy()) {
    return NO_CHANNEL;
  }
  uint8_t measured_channel;
  regs.reg.status_reg |= (1<<STATUS_READY_FLAG);
  if(last_measured == CHAN_B_GAIN_32) {
    measured_channel = CHANNEL_B;
  } else {
    measured_channel = CHANNEL_A;
  }
  hx711_chanGain_t next_measured = get_to_measure(last_measured);
  int32_t value = hx711.readChannelRaw(next_measured);
  regs.reg.raw_scale[measured_channel] = to_three_bytes(value);
  if( (value != (int32_t)0xFFFFFFFF) && hx711.isBusy()) {
    regs.reg.status_reg |= (1<<STATUS_RUN_FLAG);
  }  
  last_measured = next_measured;
  return measured_channel;
}

// set system into the sleep state 
// system wakes up when USI get data
void system_sleep() {
  ADCSRA &= ~(1<<ADEN);                    // switch Analog to Digitalconverter OFF
  sleep_cpu();                        // System actually sleeps here
  ADCSRA |= (1<<ADEN);                    // switch Analog to Digitalconverter ON
}

ISR(SIG_PIN_CHANGE) {
  // only for wakeup
}

void setup() {
  EEPROM.update(0, APP_MAGIC);
  for(uint8_t i = 0; i < sizeof(regs.raw); ++i) {
    regs.raw[i] = 0;
  }
  if(EEPROM.read(1) == EE_MAGIC) {
    regs.reg.status_reg |= (1<<STATUS_EEPROM_FLAG);
    EEPROM.get(2, regs.conf);
    eeprom_config = true;
  } else {
    regs.reg.i2c_addr = DEFAULT_I2C_SLAVE_ADDRESS;
  }
  regs.reg.i2c_addr &= 0x7F;
  regs.reg.config_reg &= ~(1<<REBOOT_FLAG);
  regs.reg.config_reg &= ~(1<<WRITE_EEPROM_FLAG);

  wdt_reset();
  MCUSR = 0;
  wdt_disable();

  TinyWireS.begin(regs.reg.i2c_addr);
  //TinyWireS.begin(DEFAULT_I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
  
  regs.reg.status_reg |= (1<<STATUS_REBOOT_FLAG);

  hx711.begin();
#ifdef HAS_RATE_PIN
  pinMode(HX711_RATE_PIN, OUTPUT);
#endif
  delay(120);
  if(!hx711.isBusy()) {
    hx711.readChannelRaw(CHAN_A_GAIN_128);
    last_measured = CHAN_A_GAIN_128;
    regs.reg.status_reg |= (1<<STATUS_READY_FLAG);
  }
  powerdown = true;
  // enable pinchange int on HX711_DT
  PCMSK |= (1<<HX711_DT);
  GIMSK |= (1<<PCIE);

  parse_config();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
}

void loop() {
  if(reparse_config) {
    reparse_config = false;
    parse_config();
  }
  if(!powerdown) {
    int8_t channel = measure();
    if(channel != NO_CHANNEL) {
      int32_t scale = from_three_bytes(regs.reg.raw_scale[channel]);
      scale -= from_three_bytes(regs.reg.tara[channel]);
      regs.reg.scale[channel] = to_three_bytes(scale);
      if(avg_ct[channel] < 255) {
        avg_ct[channel]++;
      }
      if(num_avg) {
        avg_scale_256[channel] -= avg_scale_256[channel] / min(num_avg, avg_ct[channel]);
        avg_scale_256[channel] += scale / min(num_avg, avg_ct[channel]);
      }
      regs.reg.avg_scale[channel] = to_three_bytes(avg_scale_256[channel] / 256);
    }
  }
  system_sleep();
 }