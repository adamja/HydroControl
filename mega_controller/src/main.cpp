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
    - Add watch dog timer feature
    - 
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
const char MSG_SPLITTER = '|';          // Splitter to seperate topic and message within a serial message
const char MSG_END = '$';               // Symbol to determine end of serial message
QueueArray <char> char_queue;   // Serial char buffer
QueueArray <String> msg_queue;  // Serial message buffer
int msg_count = 0;              // Messages in queue to be processed

float board_voltage = 5;        // Arduino Mega analogue input reads at 5V

// MANUAL VARIABLES
bool manual_override = false;   // Automation stops when set to true
int manual_ph_plus = 0;         // Default off
int manual_ph_minus = 0;        // Default off
int manual_fan = 0;             // Default off
int manual_solenoid = 0;        // Default off

// PH VARIABLES
float ph_setpoint = 7.0;
float ph_hyst_min;
float ph_hyst_max;
float ph_hyst_setpoint;

// WATER TEMP VARIABLES

// AIR TEMP VARIABLES
#define DHTTYPE DHT22                           // DHT 22  (AM2302)
DHT dht(pin_air_temp, DHTTYPE);                 // DHT object setup
const int air_array_size = 10;                  // Air temp readings
int air_temp_cursor = 0;                        // Number of readings
float air_temp_array[air_array_size];           // Temp reading array
float air_humidity_array[air_array_size];       // Humidity reading array
float air_temp_last = 0.0;                      // Last temp reading value
float air_humidity_last = 0.0;                  // Last humidity reading value
float air_temp_avg = 0.0;                       // 
float air_humidity_avg = 0.0;                   // 

// LIGHT VARIABLES


//FAN
float fan_hyst = 5.0;
float fan_high = 30.0;

// WATER LEVELS
int water_low = 0;
int water_med = 0;
int water_high = 0;
bool water_filling = false;

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


float calc_avg(int size, float *readings) {
    // TODO: sort array and remove outliers and take medium value instead
    float total = 0;
    float avg = 0;
	for (int i = 0; i < size; i++)
	{
		total += readings[i];
	}
    avg = total / size;
    
    return avg;
}


/*******************************************************************
# AIR TEMP                     
*******************************************************************/

void air_temp_loop() {
    // Fill array with readings
    air_temp_array[air_temp_cursor] = air_temp_last;
    air_humidity_array[air_temp_cursor] = air_humidity_last;
    air_temp_cursor++;  // increment cursor
    if (air_temp_cursor >= air_array_size) {
        air_temp_avg = calc_avg(air_array_size, air_temp_array);
        air_humidity_avg = calc_avg(air_array_size, air_humidity_array);
        air_temp_cursor = 0;  // reset cursor
    }
}


bool read_air_temp() {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();  // Read humidity
    float t = dht.readTemperature();  // Read temperature
  
    if (isnan(h) || isnan(t)) {
        return false;  // Failed to read, return false
    }
    air_temp_last = h;
    air_temp_last = t;
    return true;
}

/*******************************************************************
# WATER PH                    
*******************************************************************/

void read_water_ph() {

}

/*******************************************************************
# WATER TEMP                    
*******************************************************************/

void read_water_temp() {

}

/*******************************************************************
# LIGHT                 
*******************************************************************/

void read_light() {

}

/*******************************************************************
# SET PH                  
*******************************************************************/

void set_ph() {

}

/*******************************************************************
# FAN                    
*******************************************************************/

void set_fan() {

    air_temp_loop();

    if (air_temp_avg >= fan_high) {
        digitalWrite(pin_fan, HIGH);
    }
    else if (air_temp_avg <= (fan_high - fan_hyst)) {
        digitalWrite(pin_fan, LOW);
    }
}

/*******************************************************************
# WATER LEVELS                    
*******************************************************************/

void read_water_levels() {
    water_low = digitalRead(pin_water_low);
    water_med = digitalRead(pin_water_middle);
    water_high = digitalRead(pin_water_high);
}


void fill_water_tank() {

    read_water_levels();

    if (water_low == 1) {
        water_filling = true;
    }
    if (water_filling == true) {
        if (water_med == 1) {
            digitalWrite(pin_solenoid, LOW);
            water_filling = false;
        }
        else {
            digitalWrite(pin_solenoid, HIGH);
        }
    }
}

/*******************************************************************
# MANUAL MODE                    
*******************************************************************/

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

/*******************************************************************
# SERIAL                
*******************************************************************/

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {

    while (Serial.available()) {
      char inChar = (char)Serial.read();    // get the new byte

      if (inChar == MSG_END) {
        // msg end
        String msg = "";
        while (!char_queue.isEmpty ()) {
            msg += char_queue.dequeue();
        }
        msg_queue.enqueue(msg);
      }
      else {
        char_queue.enqueue(inChar);           // add it to the char_queue
      }
    }
    msg_count = msg_queue.count();
}


void process_msg_queue() {
    String msg = "";
    String topic = "";
    String state = "";  
    bool first = true;  // searching for the topic when true, state when false
    
    while (!msg_queue.isEmpty()) {
        msg = msg_queue.dequeue();

        // Separate message into topic and state
        for (int i = 0; i < msg.length(); i++) {
            if (msg[i] == MSG_SPLITTER) {
              first = false;
            }
            else if (first == true) {
                topic += msg[i];
            }
            else {
                state += msg[i];
            }
        }

        // Processing message
        if (topic == "manual") {
            if (state == "0") {
                manual_override = 0;
            }
            else if (state == "1") {
                manual_override = 1;
            }
        }
        if (topic == "ph_plus") {
            if (state == "0") {
                manual_ph_plus = 0;
            }
            else if (state == "1") {
                manual_ph_plus = 1;
            }
        }
        if (topic == "ph_minus") {
            if (state == "0") {
                manual_ph_minus = 0;
            }
            else if (state == "1") {
                manual_ph_minus = 1;
            }
        }
        if (topic == "fan") {
            if (state == "0") {
                manual_fan = 0;
            }
            else if (state == "1") {
                manual_fan = 1;
            }
        }
        if (topic == "solenoid") {
            if (state == "0") {
                manual_solenoid = 0;
            }
            else if (state == "1") {
                manual_solenoid = 1;
            }
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
          if (read_air_temp()) {    // Get air temp and humidity
            air_temp_loop();                // it new reading add to array
          }
          read_water_ph();               // Get water ph
          read_water_temp();             // Get water temp
          read_light();                  // Get light level
          read_water_levels();
          // send_readings();                // Write sensor readings to serial
  
          set_ph();                     // Set ph
          set_fan();                    // Set fan
          fill_water_tank();               // Set solenoid
      }
      else {
        set_manual_mode();              // Manual mode function
      }

      serialEvent();                   // Process serial received messages
      process_msg_queue();
  }