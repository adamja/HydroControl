/*
Program: Mega Hydro Control
Version: 1.0.0
Author: Adam Jacob
Comments:

TODO:
    - Interrupt for serial read. Store read values into an array to
      be read in a function all at once.
      Looking further into it, we might be able to use SerialEvent:
      https://www.arduino.cc/en/Tutorial/SerialEvent
    - Add a queue library to assist with incoming messages
      https://playground.arduino.cc/Code/QueueArray#Download
*/

/*******************************************************************
# INCLUDES                 
*******************************************************************/

#include <Arduino.h>        // Arduino library
#include <Wire.h>
#include <DHT.h>            // Temperature sensor library  for DHT22
#include <QueueArray.h>     // Queue library for holding serial input messages

/*******************************************************************
# PIN SETUP                     
*******************************************************************/
// Inputs
#define pin_water_ph A2           // Water ph sensor
#define pin_water_temp A0         // Water temp sensor
#define pin_air_temp 50           // Air temp sensor
#define pin_water_high 7          // Water high reed switch
#define pin_water_middle 8        // Water middle reed switch
#define pin_water_low 9           // Water low reed switch
#define pin_light A1              // Light sensor

// Outputs
#define pin_ph_plus 45            // ph plus relay
#define pin_ph_minus 43           // ph minus relay
#define pin_fan 49                // Fan relay
#define pin_solenoid 47           // Water solenoid relay

/*******************************************************************
# VARIABLE SETUP                 
*******************************************************************/

// SERIAL
#define BAUD_RATE 9600          // Serial Baud Rate
#define MSG_SPLITTER |          // Splitter to seperate topic and message within a serial message
#define MSG_DELIMITER $         // Delimiter to determine end of serial message

float board_voltage = 5;        // Arduino Mega analogue input reads at 5V

// MANUAL VARIABLES
bool manual_override = false;   // Automation stops when set to true
int manual_ph_plus = 0;         // Default off
int manual_ph_minus = 0;        // Default off
int manual_fan = 0;             // Default off
int manual_solenoid = 0;        // Default off

// PH VARIABLES

// WATER TEMP VARIABLES

// AIR TEMP VARIABLES
#define DHTTYPE DHT22           // DHT 22  (AM2302)
DHT dht(pin_air_temp, DHTTYPE);
float air_temp = 0.0;
float air_humidity = 0.0;       

// LIGHT VARIABLES

/*******************************************************************
# FUNCTIONS                     
*******************************************************************/

void pin_setup() {
    // Inputs
    pinMode(pin_water_ph, INPUT);
    pinMode(pin_air_temp, INPUT);
    pinMode(pin_water_high, INPUT);
    pinMode(pin_water_middle, INPUT);
    pinMode(pin_water_low, INPUT);
    pinMode(pin_light, INPUT);

    // Outputs
    pinMode(pin_ph_plus, OUTPUT);
    pinMode(pin_ph_minus, OUTPUT);
    pinMode(pin_fan, OUTPUT);
    pinMode(pin_solenoid, OUTPUT);
}


void get_air_temp_humidity() {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();
  
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.print(" *C ");
}


void get_water_ph() {

}


void get_water_temp() {

}


void get_light() {

}


void set_ph() {

}


void set_fan() {

}


void set_solenoid() {

}


void set_manual_mode() {
    // Wait for commands via serial to toggle outputs
    
    // ph minus
    if (manual_ph_minus == 1) {
        digitalWrite(pin_ph_minus, HIGH);
    }
    else {
        digitalWrite(pin_ph_minus, LOW);
    }
    // ph plus
    if (manual_ph_plus == 1) {
        digitalWrite(pin_ph_plus, HIGH);
    }
    else {
        digitalWrite(pin_ph_plus, LOW);
    }

    // fan
    if (manual_fan == 1) {
        digitalWrite(pin_fan, HIGH);
    }
    else {
        digitalWrite(pin_fan, LOW);
    }

    // solenoid
    if (manual_solenoid == 1) {
        digitalWrite(pin_solenoid, HIGH);
    }
    else {
        digitalWrite(pin_solenoid, LOW);
    }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      /* inputString += inChar; */
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      if (inChar == '\n') {
        /* stringComplete = true; */
      }
    }
}

/*******************************************************************
# SETUP AND LOOP                       
*******************************************************************/

void setup() {
    pin_setup();                  // Setup pins as inputs and outputs
    Serial.begin(BAUD_RATE);      // Start Serial
    dht.begin();                  // Air Temp
  }
  
  
  void loop() {
      if (manual_override == false) {
          get_air_temp_humidity();      // Get air temp and humidity
          get_water_ph();               // Get water ph
          get_water_temp();             // Get water temp
          get_light();                  // Get light level
          
          // send_readings();                // Write sensor readings to serial
  
          set_ph();                     // Set ph
          set_fan();                    // Set fan
          set_solenoid();               // Set solenoid
      }
      else {
        set_manual_mode();              // Manual mode function
      }

      serialEvent();                   // Process serial received messages
  }