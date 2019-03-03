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
#define MY_RF24_PA_LEVEL RF24_PA_LOW
//#define MY_RADIO_RFM69

#include <MySensors.h>

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_NODE_ID 1

#include <TimerOne.h>

#define DIMMERS_CHILD_ID_OFFSET 0 // the dimmers connected via the KRIDA board and are presented as child-sensor-id 1-8
#define DIMMERS 8
#define GATE_IMPULSE 5 // delay of triac firing in microseconds

unsigned int dimmer_pins[DIMMERS] = {3, 4, 5, 6, 7, 8, 14, 15};
unsigned char clock_cn[DIMMERS]; // how many iterations is the triac already on ?
unsigned int clock_tick; // counting the percentages (0-100) after every cross of zero in the sinus wave of AC.
unsigned int zero_cross_pin = 2;

#define LIGHT_OFF 0
#define LIGHT_ON 1

int16_t last_state[DIMMERS]; // = {LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF};
int16_t last_dim[DIMMERS]; // = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int dimmer_level[DIMMERS]; // = {95, 95, 95, 95, 95, 95, 95, 95}; // going from 5 (MAX) to 75 (LOW) and 95 (OFF)

MyMessage light_msg[DIMMERS]; // = { MyMessage( 1, V_STATUS ), MyMessage( 2, V_STATUS ), MyMessage( 3, V_STATUS ), MyMessage( 4, V_STATUS ), MyMessage( 5, V_STATUS ), MyMessage( 6, V_STATUS ), MyMessage( 7, V_STATUS ), MyMessage( 8, V_STATUS ) };
MyMessage dimmer_msg[DIMMERS]; // = { MyMessage( 1, V_PERCENTAGE ), MyMessage( 2, V_PERCENTAGE ), MyMessage( 3, V_PERCENTAGE ), MyMessage( 4, V_PERCENTAGE ), MyMessage( 5, V_PERCENTAGE ), MyMessage( 6, V_PERCENTAGE ), MyMessage( 7, V_PERCENTAGE ), MyMessage( 8, V_PERCENTAGE ) };

void zero_crosss_int() // function to be fired at the zero crossing to dim the light
{
  // Every zerocrossing interrupt: For 50Hz (1/2 Cycle) => 10ms ; For 60Hz (1/2 Cycle) => 8.33ms
  // 10ms=10000us , 8.33ms=8330us

  clock_tick = 0;
}

void timerIsr()
{
  clock_tick++;

  for (int i = 0; i < DIMMERS; i++) {
    if (clock_cn[i]) {
      clock_cn++;
  
      if (clock_cn == GATE_IMPULSE) {
        for (int i = 0; i < DIMMERS; i++) {
          digitalWrite(dimmer_pins[i], LOW); // triac firing
        }
        clock_cn[i] = 0;
      }
    }

    if (dimmer_level[i] == clock_tick) { // if the current percentage (clock_tick) is the required dimming percentage then fire the dimmer.
      digitalWrite(dimmer_pins[i], HIGH); // triac firing
      clock_cn[i] = 1;
    }
  }
}

void setup()
{
  // configure the pins used by the dimmers
  for (int i = 0; i < DIMMERS; i++) {
    pinMode(dimmer_pins[i], OUTPUT); // Set AC Load pin as output
    last_state[i] = LIGHT_OFF;
    last_dim[i] = 0;
    light_msg[i] = MyMessage(i, V_STATUS);
    dimmer_msg[i] = MyMessage(i, V_PERCENTAGE);
    clock_cn[i] = 0;
  }

  update_light();
  Serial.println( "Node ready to receive messages..." );

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

void loop()
{
  //In MySensors2.x, first message must come from within loop()
  static bool first_message_sent = false;
  if ( first_message_sent == false ) {
    Serial.println( "Sending initial state..." );
    for (int sensor = 1; sensor <= DIMMERS; sensor++) {
      send_dimmer_message(sensor);
      send_status_message(sensor);
    }
    first_message_sent = true;
  }
}

void receive(const MyMessage &message)
{
  //When receiving a V_STATUS command, switch the light between OFF
  //and the last received dimmer value  
  if ((message.sensor >= DIMMERS_CHILD_ID_OFFSET + 1) and (message.sensor <= DIMMERS_CHILD_ID_OFFSET + DIMMERS)) { // is this a message for a dimmer ?
    if ( message.type == V_STATUS ) {
      Serial.println( "V_STATUS command received..." );

      int lstate = message.getInt();
      if (( lstate < 0 ) || ( lstate > 1 )) {
        Serial.println( "V_STATUS data invalid (should be 0/1)" );
        return;
      }
      last_state[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = lstate;
  
      //If last dimmer state is 0, set dimmer to 100
      if (( last_state[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] == LIGHT_ON ) && ( last_dim[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] == 0 )) {
        last_dim[message.sensor]=100;
      }
  
      //Update constroller status
      send_status_message(message.sensor);
  
    } else if ( message.type == V_PERCENTAGE ) {
      Serial.println( "V_PERCENTAGE command received..." );
      int dim_value = constrain( message.getInt(), 0, 100 );
      if ( dim_value == 0 ) {
        last_state[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = LIGHT_OFF;
        // when light goes off set dimmer to 0 so will come on at 100% next time
        last_dim[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = 0;

        //Update constroller with dimmer value & status
        send_dimmer_message(message.sensor);
        send_status_message(message.sensor);      
      } else {
        last_state[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = LIGHT_ON;
        last_dim[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = dim_value;
  
        //Update constroller with dimmer value
        send_dimmer_message(message.sensor);
      }
  
    } else {
      Serial.println( "Invalid command received..." );
      return;
    }
  } else {
    Serial.println( "No such child ID..." );
    return;
  }

  //Here you set the actual light state/level
  update_light();

/*  Serial.println((String) "sensor: " + message.sensor + ", type: " + message.type + ", value: " + message.getString());
  if ((message.sensor >= DIMMERS_CHILD_ID_OFFSET + 1) and (message.sensor <= DIMMERS_CHILD_ID_OFFSET + DIMMERS)) { // is this a message for a dimmer ?
    if (message.type == V_STATUS) {
      if (message.getBool() == 1) {
        dimmer_level[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = 100;
      } else {
        dimmer_level[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = 900;
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
      dimmer_level[message.sensor - DIMMERS_CHILD_ID_OFFSET - 1] = level;
    }
  }*/
}

void update_light()
{
  for (int i = 0; i < DIMMERS; i++) {
    Serial.print ( "Sensor: " + i );
    Serial.print ( ", " );
    if ( last_state[i] == LIGHT_OFF ) {
      Serial.println( "Light state: OFF" );
      dimmer_level[i] = 950;
    } else {
      Serial.println( "Light state: ON, Level: " + last_dim[i] );
      dimmer_level[i] = (75 - (last_dim[i] * 70 / 100) * 10);
    }
  }
}

void send_dimmer_message(int sensor)
{
  send( dimmer_msg[sensor - 1].set( last_dim[sensor - 1] ) );
}

void send_status_message(int sensor)
{
  if ( last_state[sensor -1] == LIGHT_OFF ) {
    send( light_msg[sensor - 1].set( (int16_t)0) );
  } else {
    send( light_msg[sensor - 1].set( (int16_t)1) );
  }
}

