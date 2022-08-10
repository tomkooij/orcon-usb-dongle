#include <Arduino.h>
#include "cc1101.h"
#include "transcoder.h"

#define REQUEST_RATE                2500  // ms
#define ONBOARD_LED                 7
#define LED_FLASH_TIME              50  // ms

enum module_states
{
  JUST_BOOTED,
  LISTEN
};

CC1101 radio;

//******************************************************************************************//
//                                                                                          //
//                        Led flash                                                         //
//                                                                                          //
//******************************************************************************************//

void led_flash_once_ms(int blink_time)
{
  digitalWrite(ONBOARD_LED, LOW);  // RADIO LED ON
  delay(blink_time);
  digitalWrite(ONBOARD_LED, HIGH); // RADIO LED OFF   
}



void tty_write_str(char *s) {
  //while (*s) tty_write_char(*s++);
  Serial.print(s);
}


uint8_t manchester_decode(uint8_t a, uint8_t b) {
    uint8_t mch_lookup[16] = { 0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95, 0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55 };
    uint8_t out;
    
    for (uint8_t x = 0; x < 16; x++)
    {
      if (b == mch_lookup[x])
        out = x;
    }
  for (uint8_t x = 0; x < 16; x++)
    {
      if (a == mch_lookup[x])
        out |= x << 4;
    }
  return out;
}

//******************************************************************************************//
//                                                                                          //
//                        Init                                                              //
//                                                                                          //
//******************************************************************************************//
void setup()
{
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);

  Serial.begin(9600); // Virtual comport via USB
  while(!Serial);
  
  Serial1.begin(38400); // used for transmitting data to CC1101
  while(!Serial1);

  transcoder_init(&tty_write_str, &tty_write_str);
}

//******************************************************************************************//
//                                                                                          //
//                        MAIN                                                              //
//                                                                                          //
//******************************************************************************************//
void loop()
{
  module_states dongle_state = JUST_BOOTED;
  int prev_speed_lvl = 0;
  char rssi_str[] = "---";

  int8_t rssi;
  
  while (1)
  {
      led_flash_once_ms(1000);

    switch (dongle_state)
    {
      case (JUST_BOOTED):
        radio.init();

        if ((radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) &0x1f) == 1)
        {
          Serial.println("> CC1101 radio initialized");
          dongle_state = LISTEN;
          //dongle_state = NORMAL_MODE; //skip clone mode
        }
        else
        {
          Serial.println("> Something went wrong with radio init, will try again");
          delay(1000);
        }
        break;

      case (LISTEN):
        
        radio.set_rx_mode();
         
        while(1) { 
          

          int len_frame = 0;
          if (radio.listen_frame(15000)) {
            
            //Serial.println("\nFrame: ");
            //radio.print_rx_buffer();
            rssi = (int8_t )radio.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER);
            rssi = rssi/2 - 74;
            rssi = -rssi;
            sprintf(rssi_str, "%03u", rssi);
            Serial.println();
            Serial.print(rssi_str);
                      
            transcoder_accept_inbound_byte(0, 99); // reset transcoder (hack)
            
           uint8_t payload_byte;
           uint8_t a, b;
           for(int i=4; i<64; i++) {
              // frame starts with header: 00 33 55 53
              // skip first 4 bytes
              if (radio.rx_buffer[i] == 0x35) break;  // end of frame
              
              // manchester decode pair of two bytes
              a = radio.rx_buffer[i];
              b = radio.rx_buffer[i+1];
              payload_byte = manchester_decode(a, b);
              i++;
  
              // transcode payload.
              transcoder_accept_inbound_byte(payload_byte, 0);
            }
            
            
          } else {
            Serial.print(".");
            //radio.print_rx_buffer();
          } 
          
                  } 
    }
  }
}
