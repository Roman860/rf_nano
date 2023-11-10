#include <Arduino.h>

#include <avr/wdt.h>

#include "wiring.c"

#include <SPI.h>
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"

#include "V2RFEncoding.h"
#include <EEPROM.h>

#define V2_PACKET_LEN 9

//#define PRINT_DEBUG_LVL_1
//#define PRINT_DEBUG_LVL_2

#define WD_EN
#define SWAP_A_B // only bar lamp == swap

#ifdef SWAP_A_B
const int ch_b_on_off_pin = 4; // PD4
const int ch_b_pwm_pin = 5;    // PD5 OC0B (Timer/Counter0 output compare match B output)
const int ch_a_pwm_pin = 6;    // PD6 OC0A (Timer/Counter0 output compare match A output)
const int ch_a_on_off_pin = 7; // PD7 !!! TSTAR RED BOARD use pin 7 for CE !!!
#else
const int ch_a_on_off_pin = 4; // PD4
const int ch_a_pwm_pin = 5;    // PD5 OC0B (Timer/Counter0 output compare match B output)
const int ch_b_pwm_pin = 6;    // PD6 OC0A (Timer/Counter0 output compare match A output)
const int ch_b_on_off_pin = 7; // PD7 !!! TSTAR RED BOARD use pin 7 for CE !!!
#endif

Nrf24l Mirf = Nrf24l(10, 9); // 10-CE 9-CSN - for RF-nano (no ext ant)
// Nrf24l Mirf = Nrf24l(9, 10); //9-CE 10-CSN - for RF-nano (external antena)
// Nrf24l Mirf = Nrf24l(7, 8); // 7-CE 8-CSN - for RF-nano (TSTAR RED BOARD) and for arduino

struct device
{
  uint16_t deviceId;
  uint8_t groupId;
  bool active;
};

enum pwm_proc_sm_type
{
  initialize,
  wait_for_new_val,
  set_new_val
};

// local functions
uint8_t reverseBits(uint8_t byte);
void EEPROM_add_device(device device);
void EEPROM_update_status(uint8_t brt, uint8_t sat, bool pwr);
void EEPROM_delete_device(device device);
void blink(uint8_t num_of_blinks);
void power_up(uint8_t brt, uint8_t sat);
void power_down(uint8_t brt, uint8_t sat);
uint8_t BrtSat2PWM(uint8_t brt, uint8_t sat, bool pwm_ab);
// void delay(unsigned long ms);
////////////////////////////////////////////

// MiLight FUT089 Commands
const uint8_t FUT089_ON_OFF = 0x01;
const uint8_t FUT089_OFF = 0x01;
const uint8_t FUT089_COLOR = 0x02;
const uint8_t FUT089_BRIGHTNESS = 0x05;
const uint8_t FUT089_MODE = 0x06;
const uint8_t FUT089_KELVIN = 0x07;     // Controls Kelvin when in White mode
const uint8_t FUT089_SATURATION = 0x07; // Controls Saturation when in Color mode

// MiLight FUT089 Arguments
const uint8_t FUT089_MODE_SPEED_UP = 0x12;
const uint8_t FUT089_MODE_SPEED_DOWN = 0x13;
const uint8_t FUT089_WHITE_MODE = 0x14;

// MiLight Radio Configurations
// MiLightRadioConfig(0x147A, 0x258B, 7, 9, 40, 71, 0xAA, 0x05), // rgbw
// MiLightRadioConfig(0x050A, 0x55AA, 7, 4, 39, 74, 0xAA, 0x05), // cct
// MiLightRadioConfig(0x7236, 0x1809, 9, 8, 39, 70, 0xAA, 0x05), // rgb+cct, fut089
// MiLightRadioConfig(0x9AAB, 0xBCCD, 6, 3, 38, 73, 0x55, 0x0A), // rgb
// MiLightRadioConfig(0x50A0, 0xAA55, 6, 6, 41, 76, 0xAA, 0x0A)  // FUT020
const uint16_t syncword0 = 0x7236;
const uint16_t syncword3 = 0x1809;
const size_t packetLength = 9;
const uint8_t channel0 = 8;
const uint8_t channel1 = 39;
const uint8_t channel2 = 70;
const uint8_t preamble = 0xAA;
const uint8_t trailer = 0x05;

const uint8_t channel[3] = {channel0 + 2, channel1 + 2, channel2 + 2};

// 0 - 63 //store paired devices info (16*4)
const uint8_t EEPROM_last_paired_device_address = 64;
const uint8_t EEPROM_last_brt_address = 65;
const uint8_t EEPROM_last_sat_address = 66;
const uint8_t EEPROM_last_pwr_address = 67;

const uint8_t paired_devices_length = 16;

// const unsigned int pwm_freq = 10;                        // Hz
// const unsigned long pwm_total_time = 1000000 / pwm_freq; // in micro seconds
const long unsigned power_on_time = 3000;                // in mSec,
const long unsigned power_off_time = 5000;               // in mSec,
const long unsigned pairing_time = power_on_time + 3000; // in mSec, time after strat up while remote can be paired / unpaired
const long unsigned EEPROM_update_time = 60000;          // in mSec, steady time of brt and sat before write values to EEPROM

// globals
#if defined(PRINT_DEBUG_LVL_1) || defined(PRINT_DEBUG_LVL_2)
char *cstr = new char[128]; // for sprintf
#endif

V2RFEncoding decoder;
device paired_devices[paired_devices_length];
uint8_t payload[packetLength + 1];
uint8_t payload_size = sizeof(payload);

pwm_proc_sm_type pwm_proc_sm = initialize;
unsigned long start_time = 0;

uint8_t ch_num = 0;
uint8_t ch_a_pwm_value = 0;

uint8_t last_sequenceNum = 0;
uint8_t last_ch_a_pwm_value = 0; // read from EEPROM
bool ch_a_on_off_value = false;

boolean pairing_period = true;
uint8_t pair_count = 0;
device pair_device;

bool pair_add_new = false;
bool pair_del = false;

uint8_t brt = 0;
uint8_t sat = 0;
bool pwr = false;
uint8_t saved_brt = 0;
uint8_t saved_sat = 0;
bool saved_pwr = false;
uint8_t last_pwm_a = 0;
uint8_t last_pwm_b = 0;
bool EEPROM_update_pending = false;

bool LED_state = false;
uint8_t CS = 2; // 1-clk ,2-clk/8, 3-clk/64, 3-clk/256, 3-clk/1024  (3=default)
uint16_t CD_counter = 0;
uint32_t idle_counter = 0;
void setup()
{
#if defined(PRINT_DEBUG_LVL_1) || defined(PRINT_DEBUG_LVL_2)
  Serial.begin(9600);
#endif
  pinMode(ch_a_on_off_pin, OUTPUT);
  pinMode(ch_a_pwm_pin, OUTPUT);
  pinMode(ch_b_on_off_pin, OUTPUT);
  pinMode(ch_b_pwm_pin, OUTPUT);

  // PWM timers configuration
#ifdef PRINT_DEBUG_LVL_2
  sprintf(cstr, "TCCR0A = %02X \r\n", TCCR0A);
  Serial.write(cstr);
  sprintf(cstr, "TCCR0B = %02X \r\n", TCCR0B);
  Serial.write(cstr);
#endif

  uint8_t WGM = 3;   // 0-normal,3-Fast PWM(MAX),7-Fast PWM(TOP)
  uint8_t COM0A = 2; // 2-non-inverting mode ,3-inverting mode
  uint8_t COM0B = 2; // 2-non-inverting mode ,3-inverting mode
  // uint8_t  CS = 2;   //1-clk ,2-clk/8, 3-clk/64, 3-clk/256, 3-clk/1024
  TCCR0A = (COM0A << 6) | (COM0B << 4) | (WGM & 0x3);
  TCCR0B = ((WGM & 0x4) << (3 - 2)) | (CS);
  delay(1);
  // TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); //COM0A1 COM0A0 COM0B1 COM0B0    –     –  WGM01  WGM00
  // TCCR0B = _BV(CS01);                                           //FOC0A  FOC0B    –      –     WGM02 CS02  CS01  CS00
  // OCR0B = 200;
#ifdef PRINT_DEBUG_LVL_2
  sprintf(cstr, "TCCR0A = %02X \r\n", TCCR0A);
  Serial.write(cstr);
  sprintf(cstr, "TCCR0B = %02X \r\n", TCCR0B);
  Serial.write(cstr);
#endif

  digitalWrite(ch_a_on_off_pin, 1);
  analogWrite(ch_a_pwm_pin, 20);

  static const uint8_t SYNCWORD_LENGTH = 5;

  uint8_t syncwordBytes[SYNCWORD_LENGTH];

  size_t ix = SYNCWORD_LENGTH;
  int nrf_config_ok = 1;

  // precompute the syncword for the nRF24.  we include the fixed preamble and trailer in the
  // syncword to avoid needing to bitshift packets.  trailer is 4 bits, so the actual syncword
  // is no longer byte-aligned.
  if (SYNCWORD_LENGTH == 5)
  {
    syncwordBytes[--ix] = reverseBits(
        ((syncword0 << 4) & 0xF0) | (preamble & 0x0F));
    syncwordBytes[--ix] = reverseBits((syncword0 >> 4) & 0xFF);
    syncwordBytes[--ix] = reverseBits(((syncword0 >> 12) & 0x0F) + ((syncword3 << 4) & 0xF0));
    syncwordBytes[--ix] = reverseBits((syncword3 >> 4) & 0xFF);
    syncwordBytes[--ix] = reverseBits(
        ((syncword3 >> 12) & 0x0F) | ((trailer << 4) & 0xF0));
  }
  else
  {
    syncwordBytes[--ix] = reverseBits(syncword0 & 0xff);
    syncwordBytes[--ix] = reverseBits((syncword0 >> 8) & 0xff);
    syncwordBytes[--ix] = reverseBits(syncword3 & 0xff);
    syncwordBytes[--ix] = reverseBits((syncword3 >> 8) & 0xff);
  }

  // NRF24L setup
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();

  while (nrf_config_ok != 0)
  {

    Mirf.setRADDR(syncwordBytes); // Set receiver address
    Mirf.setTADDR(syncwordBytes);
    Mirf.payload = payload_size;
    Mirf.channel = channel[0]; // Set the used channel

    uint8_t SETUP_RETR_reg = 0x0; // Auto Retransmit, Re-Transmit disabled
    uint8_t rf_setup_reg = 0x7;   // Data Rate = 1 Mbps , TC power = 0 dBm, setup LNA gain
    uint8_t EN_AA_reg = 0x0;      // disable auto ack
    uint8_t EN_RXADDR_reg = 0x1;  // Enable data pipe 0

    Mirf.writeRegister(SETUP_RETR, &SETUP_RETR_reg, 1);
    Mirf.writeRegister(EN_AA, &EN_AA_reg, 1);
    Mirf.writeRegister(EN_RXADDR, &EN_RXADDR_reg, 1);
    Mirf.writeRegister(RF_SETUP, &rf_setup_reg, 1);

    Mirf.config(); // Set CH, payload size, power up at RX, flush RX

    // check address written
    uint8_t data[5];
    Mirf.readRegister(RX_ADDR_P0, data, 5);
    nrf_config_ok = memcmp(syncwordBytes, data, sizeof(syncwordBytes)); // 0 = the contents of both memory blocks are equal
#ifdef PRINT_DEBUG_LVL_2
    if (nrf_config_ok == 0)
      Serial.println("nrf config OK");
    else
      Serial.println("nrf config FAIL !!!");
#endif
  }

  ////////////
#ifdef PRINT_DEBUG_LVL_1
  Serial.print("syncwordBytes = ");

  for (size_t i = 0; i < 5; i++)
    Serial.print(syncwordBytes[4 - i], HEX);
  Serial.println("");
#endif

#ifdef PRINT_DEBUG_LVL_2
  for (size_t i = 0; i <= 0x17; i++)
  {
    uint8_t data[1];
    Mirf.readRegister(i, data, 1);
    Serial.print(i, HEX);
    Serial.print("=");
    Serial.println(data[0], HEX);
  }

  uint8_t data[5];
  Mirf.readRegister(RX_ADDR_P0, data, 5);
  Serial.print(RX_ADDR_P0, HEX);
  Serial.print("=");
  for (size_t i = 0; i < 5; i++)
    Serial.print(data[4 - i], HEX);

  Serial.println("");

  Mirf.readRegister(RX_ADDR_P1, data, 5);
  Serial.print(RX_ADDR_P1, HEX);
  Serial.print("=");
  for (size_t i = 0; i < 5; i++)
    Serial.print(data[4 - i], HEX);
  Serial.println("");
#endif
  /////////// eeprom ////////

  for (uint8_t i = 0; i < paired_devices_length; i++)
  {
    // read from EEPROM
    uint8_t m = i * 4;
    if (EEPROM[m + 3] == 0x01)
    {
      paired_devices[i].deviceId = (EEPROM[m] << 8) + EEPROM[m + 1];
      paired_devices[i].groupId = EEPROM[m + 2];
      paired_devices[i].active = true;
    }
    else
    {
      paired_devices[i].active = false;
    }
  }

#ifdef PRINT_DEBUG_LVL_2

  sprintf(cstr, "paired_devices : \r\n");
  Serial.write(cstr);
  for (size_t i = 0; i < paired_devices_length; i++)
  {
    if (paired_devices[i].active == true)
    {
      sprintf(cstr, "i = %i, deviceId = %04X, groupId = %02X \r\n", i, paired_devices[i].deviceId, paired_devices[i].groupId);
      Serial.write(cstr);
    }
  }
#endif

#ifdef PRINT_DEBUG_LVL_1
  Serial.print("EEPROM :");
  for (size_t i = 0; i < EEPROM.length(); i++)
  {
    if (i % 16 == 0)
      Serial.println("");

    sprintf(cstr, "%02X ", (uint8_t)EEPROM[i]);
    Serial.write(cstr);
  }
#endif

#ifdef WD_EN
  cli(); // disables all interrupts
  wdt_reset();
  // Enter Watchdog Configuration mode:
  WDTCSR |= B00011000;
  // Set Watchdog settings:
  WDTCSR = B01001110; // WDP = 0110 = 1000ms
  sei();              // enable interrupts

  // Bit Name
  // 7 WDIF interrupt flag
  // 6 WDIE Enables Interrupts
  // 5 WDP3
  // 4 WDCE enable a configuration mode that will last 4 clock cycles.
  // 3 WDE Enables system reset on time-out.
  // 2 WDP2
  // 1 WDP1
  // 0 WDP0 determine how long the timer will count for before resetting.

#endif

#ifdef PRINT_DEBUG_LVL_2
  Serial.println("start");
#endif
}

void loop()
{
  wdt_reset();

  if (idle_counter == 0x80000) // ~60sec
  {
    Mirf.powerDown();
    
    Mirf.config(); // Set CH, payload size, power up at RX, flush RX
#ifdef PRINT_DEBUG_LVL_2
    Serial.println("mirf re-config (idle)");
#endif
    idle_counter = 0;
  }
  else
  {
    idle_counter++;
  }

  if (CD_counter < 10000)
  {
    uint8_t data[1];
    Mirf.readRegister(CD, data, 1);
    if (data[0] == 1)
    {
      CD_counter++;
    }
  }
  else
  {
    CD_counter = 0;
    Mirf.config(); // Set CH, payload size, power up at RX, flush RX
#ifdef PRINT_DEBUG_LVL_2
    Serial.println("mirf re-config");
#endif
  }

  if (pairing_period)
  {
    if (millis() - start_time > pairing_time)
    {
      pairing_period = false;
#ifdef PRINT_DEBUG_LVL_2
      sprintf(cstr, "pairing period ended  \r\n");
      Serial.write(cstr);
#endif
    }
  }
  else
  {
    if ((saved_brt != brt) | (saved_sat != sat))
    {
      start_time = millis();
      EEPROM_update_pending = true;
      saved_brt = brt;
      saved_sat = sat;
    }

    else if ((EEPROM_update_pending) & (millis() - start_time > EEPROM_update_time))
    {
      EEPROM_update_pending = false;
      EEPROM_update_status(saved_brt, saved_sat, true);
    }
  }

  if (pair_add_new)
  {
    pair_add_new = false;
    pairing_period = false;
    // device new_device;
    // new_device.deviceId = deviceId;
    // new_device.groupId = groupId;
    // new_device.active = true;
#ifdef PRINT_DEBUG_LVL_2
    sprintf(cstr, "Pair new device deviceId = 0x%02X groupId = 0x%02X \r\n", pair_device.deviceId, pair_device.groupId);
    Serial.write(cstr);
#endif
    EEPROM_add_device(pair_device);
    blink(3);
  }
  else if ((pair_del) & (pair_count == 5))
  {
    pair_del = false;
    pairing_period = false;

    // device device;
    // device.deviceId = pair_deviceId;
    // device.groupId = pair_groupId;

#ifdef PRINT_DEBUG_LVL_2
    sprintf(cstr, "delete device deviceId = 0x%02X groupId = 0x%02X \r\n", pair_device.deviceId, pair_device.groupId);
    Serial.write(cstr);
#endif
    EEPROM_delete_device(pair_device);
    blink(5);
  }

  if ((saved_pwr == false) & (pwr == true)) // rising edge
  {
    saved_pwr = pwr;
#ifdef PRINT_DEBUG_LVL_1
    sprintf(cstr, "Power UP start \r\n");
    Serial.write(cstr);
#endif
    power_up(brt, sat);

#ifdef PRINT_DEBUG_LVL_1
    sprintf(cstr, "Power UP done \r\n");
    Serial.write(cstr);
#endif
  }

  if ((saved_pwr == true) & (pwr == false)) // falling edge
  {
    saved_pwr = pwr;
#ifdef PRINT_DEBUG_LVL_1
    sprintf(cstr, "Power DOWN start \r\n");
    Serial.write(cstr);
#endif
    power_down(brt, sat);
#ifdef PRINT_DEBUG_LVL_1
    sprintf(cstr, "Power DOWN done \r\n");
    Serial.write(cstr);
#endif
  }

  switch (pwm_proc_sm)
  {
  case initialize:

    start_time = millis();
    pairing_period = true;

    brt = EEPROM[EEPROM_last_brt_address];
    saved_brt = brt;

    sat = EEPROM[EEPROM_last_sat_address];
    saved_sat = sat;

    // pwr = EEPROM[EEPROM_last_pwr_address];
    pwr = true; // after power loss always turn on

    pwm_proc_sm = set_new_val;
    break;

  case wait_for_new_val:

    if (Mirf.dataReady())
    {
#ifdef PRINT_DEBUG_LVL_1
      sprintf(cstr, "Mirf.dataReady() \r\n");
      Serial.write(cstr);
#endif

      Mirf.getData(payload);

      payload[0] = reverseBits(payload[0]); // reverse first byte to check length
      uint8_t length = payload[0];          // length not encoded
      if (length != V2_PACKET_LEN)          // check packet length
      {
#ifdef PRINT_DEBUG_LVL_2
        sprintf(cstr, "wrong packet length = 0x%02X instead 0x%02X \r\n", length, V2_PACKET_LEN);
        Serial.write(cstr);
#endif
        break;
      }
      for (size_t i = 1; i < payload_size; i++) // reverse the rest of the packet
        payload[i] = reverseBits(payload[i]);

      decoder.decodeV2Packet(payload); // decode the packet , first byte = packet length , last = C.Sum

      uint8_t protocolId = payload[2];
      uint16_t deviceId = (payload[3] << 8) + payload[4];
      uint8_t command = payload[5];
      uint8_t argument = payload[6];
      uint8_t sequenceNum = payload[7];
      uint8_t groupId = payload[8];
      uint8_t Checksum = payload[9];
      uint8_t calcChecksum = decoder.checksumV2Packet(payload);
#ifdef PRINT_DEBUG_LVL_1
      uint8_t key = payload[1];
      if (sequenceNum != last_sequenceNum)
      {
        sprintf(cstr, "length      = 0x%02X \r\n", length);
        Serial.write(cstr);
        sprintf(cstr, "key         = 0x%02X \r\n", key);
        Serial.write(cstr);
        sprintf(cstr, "protocolId  = 0x%02X \r\n", protocolId);
        Serial.write(cstr);
        sprintf(cstr, "deviceId    = 0x%02X \r\n", deviceId);
        Serial.write(cstr);
        sprintf(cstr, "command     = 0x%02X \r\n", command);
        Serial.write(cstr);
        sprintf(cstr, "argument    = 0x%02X \r\n", argument);
        Serial.write(cstr);
        sprintf(cstr, "sequenceNum = 0x%02X \r\n", sequenceNum);
        Serial.write(cstr);
        sprintf(cstr, "groupId     = 0x%02X \r\n", groupId);
        Serial.write(cstr);
        sprintf(cstr, "Checksum    = 0x%02X \r\n", Checksum);
        Serial.write(cstr);
        sprintf(cstr, "calc C.Sum  = 0x%02X \r\n", calcChecksum);
        Serial.write(cstr);
      }
#endif

      if (Checksum != calcChecksum) // bad checksum
      {
#ifdef PRINT_DEBUG_LVL_2
        sprintf(cstr, "bad checksum: 0x%02X instead 0x%02X \r\n", Checksum, calcChecksum);
        Serial.write(cstr);
#endif
        break;
      }

      if (protocolId != 0x25) // protocol is 0x25
      {
#ifdef PRINT_DEBUG_LVL_2
        sprintf(cstr, "wrong protocolId = 0x%02X instead 0x25 \r\n", protocolId);
        Serial.write(cstr);
#endif
        break;
      }

      if (sequenceNum != last_sequenceNum) // verify it's a new message
        last_sequenceNum = sequenceNum;
      else
        break;

      if (command == FUT089_ON_OFF)
      {
        if (argument <= 8)
          groupId = argument;
        else if (argument <= 17)
          groupId = argument - 9;
      }
      // up until here we verified the message is correct
      // now we will check if the message intend to us
      bool relavnt_packet = false;
      for (uint8_t i = 0; i < paired_devices_length; i++)
      {
        if (paired_devices[i].active)
          if ((paired_devices[i].deviceId == deviceId)) // stored device ID
          {
            if ((groupId == 0) | (paired_devices[i].groupId == groupId)) // broadcast or stored group ID
            {
              relavnt_packet = true;
              break;
            }
          }
      }

      if ((!relavnt_packet) & (command == FUT089_ON_OFF) & (pairing_period)) // the message not relevant but maybe we need to pair new device
      {

        if (pair_count == 0)
        {
          pair_device.deviceId = deviceId;
          pair_device.groupId = groupId;
          pair_device.active = true;

          pair_count++;
        }
        else if ((pair_count < 3) & (pair_device.deviceId == deviceId) & (pair_device.groupId == groupId))
        {
          pair_count++;
        }
        else if (pair_count == 3)
        {
          pair_count++;
          pair_add_new = true;
        }
      }

      if (!relavnt_packet)
      {
#ifdef PRINT_DEBUG_LVL_2
        sprintf(cstr, "packet not relevant groupId = 0x%02X \r\n", groupId);
        Serial.write(cstr);
#endif
        break; // any way don't pross the message
      }

      switch (command)
      {

      case FUT089_ON_OFF:
        if (argument <= 8) // on
        {
          pwr = true;
#ifdef PRINT_DEBUG_LVL_2
          sprintf(cstr, "power on \r\n");
          Serial.write(cstr);
#endif
          pwm_proc_sm = set_new_val;

          if (pairing_period)
          {
            pair_count++;
            pair_del = true;
            pair_device.deviceId = deviceId;
            pair_device.groupId = groupId;
          }
        }
        else if (argument <= 17) // off
        {
          pwr = false;
#ifdef PRINT_DEBUG_LVL_2
          sprintf(cstr, "power off \r\n");
          Serial.write(cstr);
#endif
          pwm_proc_sm = set_new_val;
        }

        break;
      case FUT089_BRIGHTNESS:
        if (pwr)
        {
          brt = argument;
          pwm_proc_sm = set_new_val;
        }
        break;

      case FUT089_SATURATION:
        if (pwr)
        {
          sat = argument;
          pwm_proc_sm = set_new_val;
        }
        break;

      default:
        break;
      }
    }

    // uint8_t cd;
    // Mirf.readRegister(CD,&cd,1);
    // if (cd == 1)
    // {
    //   Serial.println("CD !");
    // }

    // if (ch_num >= 2)
    //   ch_num = 0;
    // else
    //   ch_num++;

    // //Mirf.ceLow();
    // Mirf.configRegister(RF_CH, channel[ch_num]); //switch channel 10
    // //Mirf.ceHi();

    // delay(100);

    break;

  case set_new_val:

    uint8_t pwm_a = BrtSat2PWM(brt, sat, true);
    uint8_t pwm_b = BrtSat2PWM(brt, sat, false);

    // OCR0B = ch_a_pwm_value;
    // if (sat < 50)
    //  {
    //    pwm_a = brt * (sat / 50.0) * 255.0 / 100.0;
    //    pwm_b = brt * 255 / 100;
    //  }
    //  else
    //  {
    //    pwm_a = brt * 255 / 100;
    //    pwm_b = brt * ((100 - sat) / 50.0) * 255.0 / 100.0;
    //  }

#ifdef PRINT_DEBUG_LVL_1
    sprintf(cstr, "set_new_val, brt = %d  sat = %d\r\n", brt, sat);
    Serial.write(cstr);
    sprintf(cstr, "pwm_a = %d  pwm_b = %d\r\n", pwm_a, pwm_b);
    Serial.write(cstr);
#endif

    digitalWrite(ch_a_on_off_pin, pwr);
    digitalWrite(ch_b_on_off_pin, pwr);

    if (pwr == false)
    {
      analogWrite(ch_a_pwm_pin, 0);
      analogWrite(ch_b_pwm_pin, 0);
    }
    else
    {

      while ((last_pwm_a != pwm_a) | (last_pwm_b != pwm_b))
      {
        if (last_pwm_a < pwm_a) // pwm_a inc
          last_pwm_a++;
        else if (last_pwm_a > pwm_a) // pwm_a dec
          last_pwm_a--;

        if (last_pwm_b < pwm_b) // pwm_b inc
          last_pwm_b++;
        else if (last_pwm_b > pwm_b) // pwm_b dec
          last_pwm_b--;

        analogWrite(ch_a_pwm_pin, last_pwm_a);
        analogWrite(ch_b_pwm_pin, last_pwm_b);
        delay(1);
        wdt_reset();
      }
    }

    pwm_proc_sm = wait_for_new_val;

    break;

    // default:
    //   break;
  }
}

uint8_t reverseBits(uint8_t byte)
{
  uint8_t result = byte;
  uint8_t i = 7;

  for (byte >>= 1; byte; byte >>= 1)
  {
    result <<= 1;
    result |= byte & 1;
    --i;
  }

  return result << i;
}

void EEPROM_add_device(device device)
{
  uint8_t device_address = EEPROM[EEPROM_last_paired_device_address]; // last device address
  device_address = (device_address + 1) % 15;
  EEPROM[device_address * 4 + 0] = device.deviceId >> 8; // MSB
  EEPROM[device_address * 4 + 1] = device.deviceId;      // LSB
  EEPROM[device_address * 4 + 2] = device.groupId;
  EEPROM[device_address * 4 + 3] = device.active;

  EEPROM[64] = device_address;

  paired_devices[device_address] = device;
}

void EEPROM_delete_device(device device)
{

#ifdef PRINT_DEBUG_LVL_2
  sprintf(cstr, "EEPROM deleting device groupId = 0x%02X deviceId = 0x%02X \r\n", device.groupId, device.deviceId);
  Serial.write(cstr);
#endif

  uint8_t device_address = paired_devices_length;
  for (size_t i = 0; i < paired_devices_length; i++)
  {
    if ((paired_devices[i].deviceId == device.deviceId) & (paired_devices[i].groupId == device.groupId) & (paired_devices[i].active == true))
    {
      device_address = i;
      break;
    }
  }

  if (device_address != paired_devices_length)
  {
    EEPROM[device_address * 4 + 3] = false;
    paired_devices[device_address].active = false;

#ifdef PRINT_DEBUG_LVL_2
    sprintf(cstr, "EEPROM deleted device device_address = 0x%02X \r\n", device_address);
    Serial.write(cstr);
#endif
  }
}

void EEPROM_update_status(uint8_t brt, uint8_t sat, bool pwr)
{
  EEPROM[EEPROM_last_brt_address] = brt;
  EEPROM[EEPROM_last_sat_address] = sat;
  // EEPROM[EEPROM_last_pwr_address] = pwr;
#ifdef PRINT_DEBUG_LVL_2
  sprintf(cstr, "write to EEPROM brt = 0x%02X sat = 0x%02X \r\n", brt, sat);
  Serial.write(cstr);
#endif
}

void blink(uint8_t num_of_blinks)
{

  while (num_of_blinks--)
  {
    for (uint8_t i = 0; i < 255; i++)
    {
      analogWrite(ch_a_pwm_pin, i);
      analogWrite(ch_b_pwm_pin, i);
      delay(2);
      wdt_reset();
    }

    for (uint8_t i = 255; i > 0; i--)
    {
      analogWrite(ch_a_pwm_pin, i);
      analogWrite(ch_b_pwm_pin, i);
      delay(2);
      wdt_reset();
    }
  }

  analogWrite(ch_a_pwm_pin, 255);
  analogWrite(ch_b_pwm_pin, 255);
}

uint8_t BrtSat2PWM(uint8_t brt, uint8_t sat, bool pwm_ab)
{
  uint8_t pwm_a = 0;
  uint8_t pwm_b = 0;

  // OCR0B = ch_a_pwm_value;
  if (sat < 50)
  {
    pwm_a = brt * (sat / 50.0) * 255.0 / 100.0;
    pwm_b = brt * 255 / 100;
  }
  else
  {
    pwm_a = brt * 255 / 100;
    pwm_b = brt * ((100 - sat) / 50.0) * 255.0 / 100.0;
  }

#ifdef PRINT_DEBUG_LVL_1
  sprintf(cstr, "brt = %d  sat = %d\r\n", brt, sat);
  Serial.write(cstr);
  sprintf(cstr, "pwm_a = %d  pwm_b = %d\r\n", pwm_a, pwm_b);
  Serial.write(cstr);
#endif

  if (pwm_ab)
    return (pwm_a);
  else
    return (pwm_b);

  // digitalWrite(ch_a_on_off_pin, pwr);
  // digitalWrite(ch_b_on_off_pin, pwr);

  // if (pwr == false)
  //   analogWrite(ch_a_pwm_pin, 0);
  // else
  //   analogWrite(ch_a_pwm_pin, pwm_a);

  // if (pwr == false)
  //   analogWrite(ch_b_pwm_pin, 0);
  // else
  //   analogWrite(ch_b_pwm_pin, pwm_b);

  // pwm_proc_sm = wait_for_new_val;
}

void power_up(uint8_t brt, uint8_t sat)
{
  digitalWrite(ch_a_on_off_pin, 1);
  digitalWrite(ch_b_on_off_pin, 1);

  for (uint8_t i = 0; i < brt; i++)
  {
    last_pwm_a = BrtSat2PWM(i, sat, true);
    last_pwm_b = BrtSat2PWM(i, sat, false);
    analogWrite(ch_a_pwm_pin, last_pwm_a);
    analogWrite(ch_b_pwm_pin, last_pwm_b);
    delay(power_on_time / 255);
    wdt_reset();
#ifdef PRINT_DEBUG_LVL_2
    Serial.print(".");
#endif
  }
#ifdef PRINT_DEBUG_LVL_2
  Serial.println();
#endif
}

void power_down(uint8_t brt, uint8_t sat)
{
  digitalWrite(ch_a_on_off_pin, 1);
  digitalWrite(ch_b_on_off_pin, 1);

  for (uint8_t i = brt; i > 0; i--)
  {
    last_pwm_a = BrtSat2PWM(i, sat, true);
    last_pwm_b = BrtSat2PWM(i, sat, false);
    analogWrite(ch_a_pwm_pin, last_pwm_a);
    analogWrite(ch_b_pwm_pin, last_pwm_b);
    delay(power_off_time / 255);
    wdt_reset();
#ifdef PRINT_DEBUG_LVL_2
    Serial.print(".");
#endif
  }
#ifdef PRINT_DEBUG_LVL_2
  Serial.println();
#endif
}

// void delay(unsigned long ms)
// {
//   //CS =  1-clk ,2-clk/8, 3-clk/64, 4-clk/256, 5-clk/1024
//   //       /64     /8        def        *4        *16
//   switch (CS)
//   {
//   case 1: ms = ms << 6; break;
//   case 2: ms = ms << (3+2); break;
//   case 3: break;
//   case 4: ms = ms >> 2; break;
//   case 5: ms = ms >> 4; break;

//   default:
//     break;
//   }

// 	uint32_t start = micros();

// 	while (ms > 0) {
// 		yield();
// 		while ( ms > 0 && (micros() - start) >= 1000) {
// 			ms--;
// 			start += 1000;
// 		}
// 	}
// }