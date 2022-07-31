#include <Arduino.h>
#include "cc1101.h"
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

        Serial.println("> Listening to evohome frames");
        if (radio.listen_frame(5000)) {
          Serial.println("Frame: ");
          radio.print_rx_buffer();
        } else {
          Serial.println("> Timeout !! retry ");
          radio.print_rx_buffer();
        } 
        delay(1000);
        break;
    }
  }
}
