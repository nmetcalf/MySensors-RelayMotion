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
   Version 1.1 - HenryWhite

   DESCRIPTION
   Example sketch showing how to control physical relays.
   This example will remember relay state after power failure.
   Optional attachment of motion sensor to control the relays is possible.
   Notes:
      -- The Child-IDs of the attached relays range from 1 up to (1-(NUMBER_OF_RELAYS))
      -- Make sure to adjust the potentiometer for triggertime on your motion sensor as leftmost as possible,
         because the countdown will not start until the motion sensor reports back a "0" (no movement)

*/

//----------------------- Library Configuration ---------------------
#define MY_DEBUG                          // uncomment to enable debug prints to serial monitor
#define MY_REPEATER_FEATURE               // uncomment to enable repeater functionality for this node
#define MY_NODE_ID 210                     // uncomment to define static node ID

// Enable and uncomment attached radio type
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10

//----------------------- Relay and Motion Sensor Configuration -----------------------
#define MOTION                                                    // un-comment to enable motion sensing
#define NUMBER_OF_RELAYS  3                                       // Total number of attached relays. Must be equal to total number of elements in array below!
const int RELAYS[]                  =     {5,  6,   7};           // digital pins of attached relays
const int MOTION_ACTIVATED_RELAYS[] =     {1,  0,   0};           // 1 to trigger the relay through motion, 0 to not trigger. Array length must correspond to RELAYS[] array.
const long ON_TIMES[]               =     {300, 300,  300};       // Specify for each element in MOTION_ACTIVATED_RELAYS, how long the specified relay should be active in seconds.
#define RELAY_ON          0                                       // GPIO value to write to turn on attached relay
#define RELAY_OFF         1                                       // GPIO value to write to turn off attached relay
#define MOTION_PIN        8                                       // The digital input pin of the motion sensor
#define MOTION_CHILD_ID   0                                       // Set the child id of the motion sensor
bool ack = 1;                                                     // set this to 1 if you want destination node to send ack back to this node

#define CHILD_ID_HUM 5
#define CHILD_ID_TEMP 6

// DHT11 STUFF
// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 4

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

long previousMillis = 0;        // will store last time LED was updated
//----------------------- DO NOT CHANGE -----------------------------
#include <MySensors.h>
#include <DHT.h>

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage motion_msg(MOTION_CHILD_ID, V_TRIPPED);   // Initialize motion message
unsigned long trigger_millis[NUMBER_OF_RELAYS];     // Used for the timer
bool lastTripped = 0;                               // Used to store last motion sensor value
bool manual_override = 0;                           // if this gets set to 1 (e.g. by a switch or a command from the gateway), motion triggering of relays is deactivated
MyMessage relay_msg;                                // Initialize relay message

DHT dht;


float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

void before()
{
  int i;
  for (int sensor = 1, i = 0; sensor <= NUMBER_OF_RELAYS; sensor++, i++) {
    // set relay pins to output mode
    pinMode(RELAYS[i], OUTPUT);
    // Restore relay to last known state (using eeprom storage)
    digitalWrite(RELAYS[i], loadState(sensor) ? RELAY_ON : RELAY_OFF);
  }
  // set motion pin to output mode, if MOTION is defined
#ifdef MOTION
  pinMode(MOTION_PIN, INPUT);
#endif
}

void setup()
{
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
  
#ifdef MOTION
  // give the motion sensor some time to settle
  Serial.println("Starting up. Please wait 8 seconds...");
  delay(8000);
#endif
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("NDM.Guru Office Motion", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  for (int sensor = 1; sensor <= NUMBER_OF_RELAYS; sensor++) {
    present(sensor, S_BINARY, "Relay", ack);
  }
#ifdef MOTION
  present(MOTION_CHILD_ID, S_MOTION, "Motion Sensor", ack);
#endif

}

void loop()
{
#ifdef MOTION
  if (!manual_override) {
    // Read digital motion value
    bool tripped = digitalRead(MOTION_PIN) == HIGH;

    if (lastTripped != tripped) {
      Serial.print("New Motion State: ");
      Serial.println(tripped);
      // Send tripped value to gw
      send(motion_msg.set(tripped ? "1" : "0"));
      lastTripped = tripped;

      // Change relay states, send new state to gw and store state in eeprom
      if (tripped == 1) {
        for (int i = 0; i < NUMBER_OF_RELAYS; i++) {
          if (MOTION_ACTIVATED_RELAYS[i] == 1) {
            digitalWrite(RELAYS[i], RELAY_ON);
            relay_msg_constructor(i + 1, V_STATUS);
            send(relay_msg.set(1));
            trigger_millis[i] = millis();
            Serial.print("Relay ");
            Serial.print(RELAYS[i]);
            Serial.println(" turned on");
            saveState(i, 1);
          }
        }
      }
    }

    for (int i = 0; i < NUMBER_OF_RELAYS; i++) {
      if (tripped == 1 and MOTION_ACTIVATED_RELAYS[i] == 1 and trigger_millis[i] != 0) {
        trigger_millis[i] = millis();
      }
      if ((trigger_millis[i] + ON_TIMES[i] * 1000 < millis()) and MOTION_ACTIVATED_RELAYS[i] == 1 and trigger_millis[i] != 0) {
        digitalWrite(RELAYS[i], RELAY_OFF);
        relay_msg_constructor(i + 1, V_STATUS);
        send(relay_msg.set(0));
        Serial.print("Relay ");
        Serial.print(RELAYS[i]);
        Serial.println(" turned off");
        saveState(i, 0);
        trigger_millis[i] = 0;
      }
    }
  }
  else {
    bool tripped = digitalRead(MOTION_PIN) == HIGH;
    if (lastTripped != tripped) {
      Serial.print("New Motion State: ");
      Serial.println(tripped);
      // Send tripped value to gw
      send(motion_msg.set(tripped ? "1" : "0"));
      lastTripped = tripped;
    }
    for (int i = 0; i < NUMBER_OF_RELAYS; i++) {
      if (MOTION_ACTIVATED_RELAYS[i] == 1 and trigger_millis[i] != 0) {
        trigger_millis[i] = 0;                            // reset running timers
      }
    }
  }
#endif

// Temp Stuffs - report every X minutes

unsigned long currentMillis = millis();
 
if(currentMillis - previousMillis > UPDATE_INTERVAL) {
  // Force reading sensor, so it works also after sleep()
    dht.readSensor(true);
  
    // Get temperature from DHT library
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT!");
    } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
      // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      if (!metric) {
        temperature = dht.toFahrenheit(temperature);
      }
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      temperature += SENSOR_TEMP_OFFSET;
      send(msgTemp.set(temperature, 1));
  
      #ifdef MY_DEBUG
      Serial.print("Temperature: ");
      Serial.println(temperature);
      #endif
    } else {
      // Increase no update counter if the temperature stayed the same
      nNoUpdatesTemp++;
    }
  
  delay(10);
  
    // Get humidity from DHT library
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
    } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity, 1));
  
      #ifdef MY_DEBUG
      Serial.print("Humidity: ");
      Serial.println(humidity);
      #endif
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
     previousMillis = currentMillis;   
  }
  
}

void receive(const MyMessage &message)
{
  // Handle incoming relay commands
  if (message.type == V_STATUS) {
    // Change relay state
    if (RELAYS[message.sensor - 1]) {
      digitalWrite(RELAYS[message.sensor - 1], message.getBool() ? RELAY_ON : RELAY_OFF);

      // Store state in eeprom
      saveState(message.sensor - 1, message.getBool());
      // Write some debug info
      Serial.print("Incoming change for sensor:");
      Serial.print(message.sensor);
      Serial.print(", New status: ");
      Serial.println(message.getBool());
    }
  }

  // Handle incoming manual override/bypass of motion sensor
  if (message.type == V_ARMED and message.sensor == 0) {
    manual_override = message.getBool();
    Serial.print("Manual Override: ");
    Serial.println(manual_override);
  }
}

void relay_msg_constructor(int sensor, uint8_t type)
{
  relay_msg.setSensor(sensor);
  relay_msg.setType(type);
}
