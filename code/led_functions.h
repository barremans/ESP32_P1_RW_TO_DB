#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H

#include <Arduino.h>

class LED_Controls {
public:
  LED_Controls(int pin, int delay);
  void ON();
  void OFF();
  void ESP_RUN();
  int _pin;
  int _delay;
};


class LED_Status {
public:
  LED_Status(unsigned long previousM);
  unsigned long _previousM;
  int _ledState;
  int _led_gpio = 32;
  const long _wifitimer = 1000;
};


#endif
