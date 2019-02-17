/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik Ekblad

   DESCRIPTION
   Example sketch showing how to control physical relays.
   This example will remember relay state after power failure.
   http://www.mysensors.org/build/relay
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_RF24_PA_LEVEL RF24_PA_LOW

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_NODE_ID 1

#include <MySensors.h>
#include <TimerOne.h>

#define DIMMERS_CHILD_ID_OFFSET 0 // the dimmers connected via the KRIDA board and are presented as child-sensor-id 201-208
#define DIMMERS 8
#define GATE_IMPULSE 5 // delay of triac firing
unsigned int dimmer_pins[DIMMERS] = {3, 4, 5, 6, 7, 8, 14, 15};
unsigned int dimmer_levels[DIMMERS] = {0, 0, 0, 0, 0, 0, 0, 0}; // going from 5 (MAX) to 90 (LOW) and 95 (OFF)
unsigned char clock_cn; // how many iterations is the triac already on ?
unsigned int clock_tick; // counting the percentages (0-100) after every cross of zero in the sinus wave of AC.
unsigned int zero_cross_pin = 2;

void setup()
{
  // configure the pins used by the dimmers
  for (int i = 1; i <= DIMMERS; i++) {
    pinMode(dimmer_pins[i - 1], OUTPUT); // Set AC Load pin as output
  }
  attachInterrupt(digitalPinToInterrupt(zero_cross_pin), zero_crosss_int, RISING); // every time the sinus wave of AC passes zero, the clock_tick will start from 0;
  Timer1.initialize(10); // set a timer of length 100 microseconds for 50Hz. To get to 100 slices of the sinus wave of AC.
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  Serial.begin(115200);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("MarcEnChristel Dimmers", "1.0");

  for (int sensor = 1; sensor <= DIMMERS; sensor++) {
    present(DIMMERS_CHILD_ID_OFFSET + sensor, S_DIMMER);
  }
}

void zero_crosss_int() // function to be fired at the zero crossing to dim the light
{
  // Every zerocrossing interrupt: For 50Hz (1/2 Cycle) => 10ms ; For 60Hz (1/2 Cycle) => 8.33ms
  // 10ms=10000us , 8.33ms=8330us

  clock_tick = 0;
}

void timerIsr()
{
  clock_tick++;

  if (clock_cn) {
    clock_cn++;

    if (clock_cn == GATE_IMPULSE) {
      for (int i = 0; i < DIMMERS; i++) {
        digitalWrite(dimmer_pins[i], LOW); // triac firing
      }
      clock_cn = 0;
    }
  }

  for (int i = 0; i < DIMMERS; i++) {
    if (dimmer_levels[i] == clock_tick) { // if the current percentage (clock_tick) is the required dimming percentage then fire the dimmer.
      digitalWrite(dimmer_pins[i], HIGH); // triac firing
      clock_cn = 1;
    }
  }
}

void loop()
{

}

void receive(const MyMessage &message)
{
  Serial.println((String) "sensor: " + message.sensor + ", type: " + message.type + ", value: " + message.getString());
  if ((message.sensor >= DIMMERS_CHILD_ID_OFFSET + 1) and (message.sensor <= DIMMERS_CHILD_ID_OFFSET + DIMMERS)) { // is this a message for a dimmer ?
    if (message.type == V_STATUS) {
      if (message.getBool() == 1) {
        dimmer_levels[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = 100;
      } else {
        dimmer_levels[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = 900;
      }
    }
    if (message.type == V_PERCENTAGE) {
      int level = message.getUInt();
      level = 100 - level;
      if (level < 5) {
        level = 5;
      }
      if (level > 90) {
        level = 90;
      }
      level = level * 10; // timer has been reduced from 100 to 10 microseconds
      dimmer_levels[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = level;
    }
  }
}

