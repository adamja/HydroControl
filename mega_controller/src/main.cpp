/*
Program: Mega Hydro Control
Version: 1.0.0
Author: Adam Jacob
Comments:

TODO:
    - Fix the average calculation to work with a std deviation
    - Send error messages for fault states
        - WDT trip
        - Water level conflict
    - Add a timeout period to revert test and manual modes to off.
      In the event they are left set they will deactivate and operations
      will return to normal.
*/

/*******************************************************************
# INCLUDES
*******************************************************************/

#include <Arduino.h>        // Arduino library
#include <Wire.h>
#include <avr/wdt.h>        // Watch dog timer
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
#define pin_ph_plus 4            // ph plus relay
#define pin_ph_minus 5           // ph minus relay
#define pin_fan 6                // Fan relay
#define pin_solenoid 7           // Water solenoid relay

/*******************************************************************
# VARIABLE SETUP
*******************************************************************/
// WATCH DOG TIMER
#define WDT_TIMEOUT WDTO_4S         // Watch dog timer delay time set to 4s

// SERIAL
#define BAUD_RATE 9600              // Serial Baud Rate
#define BAUD_RATE1 115200             // Serial1 Baud Rate
const char MSG_SPLITTER = '|';      // Splitter to seperate topic and message within a serial message
const char MSG_END = '$';           // Symbol to determine end of serial message
QueueArray <char> char_queue;       // Serial char buffer
String topic = "";                  
String msg = "";
bool send_all_readings = true;

// Heartbeart
unsigned long heartbeat_delay = 60000;      // Send keep heartbeat message at this interval (ms)
unsigned long heartbeat_prev_millis = 0;
unsigned long heartbeat_count = 0;

// BOARD
float board_voltage = 5;        // Arduino Mega analogue input reads at 5V

// MANUAL VARIABLES
bool manual_mode = false;   // Automation stops when set to true
int manual_ph_plus = 0;         // Default off
int manual_ph_minus = 0;        // Default off
int manual_fan = 0;             // Default off
int manual_solenoid = 0;        // Default off

// TEST VARIABLES
bool test_mode = false;

// PH VARIABLES
int ph_mode = 0;                    // 0 = idle | 1 = ph low | 2 = ph high
unsigned long ph_reading_delay = 100;   // Delay between reads
unsigned long ph_read_prev_millis = 0;  // Previous millis when reading
unsigned long ph_set_prev_millis = 0;   // Previous millis for setting duty cycle
float ph_setpoint = 7.0;            // Target value for the ph to be set to
float ph_hyst = 2.0;                // Acceptable deviation from the set point
const int ph_array_size = 10;       // Amount of readings to take before taking the average
int ph_cursor = 0;                  // Cursor for array position
float ph_array[ph_array_size];      // Init the array base on the size
float ph_last = 0.0;                // 
float ph_avg = ph_setpoint;         // Variable to store that average ph. Init with the setpoint so nothing is triggered until readings are taken
int ph_pin_state = LOW;             // Variable to store the state of pin. Used for the duty cycle / toggling when ph is on or off
unsigned long ph_pin_low_time = 10000;  // ph motor off for this time in duty cycle (ms)
unsigned long ph_pin_high_time = 1000;  // ph motor on for this time in duty cycle (ms)
unsigned long ph_pin_time = ph_pin_high_time;   // Variable to store how long to keep the high or low mode active before switching mode


// WATER TEMP VARIABLES
unsigned long water_reading_delay = 100;
unsigned long water_prev_millis = 0;
const int water_array_size = 10;
int water_cursor = 0;
float water_array[water_array_size];
float water_last = 0.0;
float water_avg = 0.0;

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
unsigned long air_reading_delay = 2500;          // Time between readings (ms)
unsigned long air_prev_millis = 0;              // 

// LIGHT VARIABLES
unsigned long light_reading_delay = 100;
unsigned long light_prev_millis = 0;
const int light_array_size = 10;
int light_cursor = 0;
float light_array[light_array_size];
float light_last = 0.0;
float light_avg = 0.0;

//FAN
float fan_hyst = 5.0;
float fan_high = 30.0;

// WATER LEVELS
int water_low_norm_state = 1;
int water_med_norm_state = 1;
int water_high_norm_state = 1;
int water_low = water_low_norm_state;
int water_med = water_med_norm_state;
int water_high = water_high_norm_state;
bool water_filling = false;

// PREVIOUS STATES
int fan_prev = 0;          // 0 = off | 1 = on
int ph_prev = 0;           // 0 = idle | 1 = ph low | 2 = ph high
int solenoid_prev = 0;     // 0 = off | 1 = on

bool test_mode_prev = test_mode;

bool manual_mode_prev = manual_mode;
int manual_ph_plus_prev = manual_ph_plus;
int manual_ph_minus_prev = manual_ph_minus;
int manual_fan_prev = manual_fan;
int manual_solenoid_prev = manual_solenoid;

int ph_plus_relay_prev = 0;
int ph_minuis_relay_prev = 0;
int fan_relay_prev = 0;
int solenoid_relay_prev = 0;

float air_temp_avg_prev = air_temp_avg;
float air_humidity_avg_prev = air_humidity_avg;
float water_avg_prev = water_avg;
float ph_avg_prev = ph_avg;
float ph_setpoint_prev = ph_setpoint;
float ph_hyst_prev = ph_hyst;
unsigned long ph_pin_low_time_prev = ph_pin_low_time;
unsigned long ph_pin_high_time_prev = ph_pin_high_time;
float light_avg_prev = light_avg;
int water_low_prev = water_low;
int water_med_prev = water_med;
int water_high_prev = water_high;
float fan_hyst_prev = fan_hyst;
float fan_high_prev = fan_high;
unsigned long heartbeat_delay_prev = heartbeat_delay;
unsigned long ph_reading_delay_prev = ph_reading_delay;
unsigned long water_reading_delay_prev = water_reading_delay;
unsigned long air_reading_delay_prev = air_reading_delay;
unsigned long light_reading_delay_prev = light_reading_delay;


/*******************************************************************
# FUNCTIONS
*******************************************************************/

void pin_setup() {
    // Inputs
    pinMode(pin_water_ph, INPUT);
    pinMode(pin_air_temp, INPUT);
    pinMode(pin_water_high, INPUT_PULLUP);
    pinMode(pin_water_middle, INPUT_PULLUP);// INPUT_PULLUP is for testing? Matt
    pinMode(pin_water_low, INPUT_PULLUP);
    pinMode(pin_light, INPUT);

    // Outputs
    pinMode(pin_ph_plus, OUTPUT);
    pinMode(pin_ph_minus, OUTPUT);
    pinMode(pin_fan, OUTPUT);
    pinMode(pin_solenoid, OUTPUT);
    digitalWrite(pin_ph_plus, LOW);
    digitalWrite(pin_ph_minus, LOW);
    digitalWrite(pin_fan, LOW);
    digitalWrite(pin_solenoid, LOW);
}

void watch_dog_timer_setup() {
    wdt_disable();
    wdt_enable(WDT_TIMEOUT);  // Set watchdog timeout value
}

float calc_avg(int size, float *readings) {
    // TODO: sort array and remove outliers and take medium value instead
    float total = 0;
    float avg = 0;

    // Bubble sort
    int temp = 0;
    for (int i = 0; i < (size - 1); i++) {
        for (int j = i + 1; j < size; j++) {
            if (readings[i] > readings[j]) {
                temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            } 
        }
    }
    // take 20% off either end of the array matt: better way to do it is calculate standard deviation (SD), then remove evrything outside some integer multiple of SD.
    /* // finds the average
    for (int i= 0; i=size-1; i++)
    {
        total + =readings[i];
    }
    avg = total / size
      // does the summing part of finding the SD
    for (int i=0; i=size-1; i++)
    {
        sum += (readings[i]-avg)^2
    }
    // calculates the SD using the previous summing
    SD =sqrt(sum / size)
    // now just have to remove all values less than avg-n*SD and larger than avg+n*SD, where n is some integer.
    */
    float percent = 0.2;
    int outlier = (int)(size * percent);

	for (int i = (0 + outlier); i < (size - outlier); i++)
	{
		total += readings[i];
	}
    avg = total / (size - (2 * outlier));
    
    return avg;
}


/*******************************************************************
# AIR TEMP
*******************************************************************/

bool read_air_temp() {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

    unsigned long cur_millis = millis();

    if (air_prev_millis < cur_millis - air_reading_delay) {
        float h = dht.readHumidity();           // Read humidity
        float t = dht.readTemperature();        // Read temperature
        
    
        if (isnan(h) || isnan(t)) {
            return false;  // Failed to read, return false
        }
        air_humidity_last = h;
        air_temp_last = t;
        air_prev_millis = millis();
        return true;
    }
    return false;
}

bool air_temp_update() {
    if (read_air_temp() == true) {
        air_temp_array[air_temp_cursor] = air_temp_last;
        air_humidity_array[air_temp_cursor] = air_humidity_last;
        air_temp_cursor++;  // increment cursor
        if (air_temp_cursor >= air_array_size) {
            air_temp_avg = calc_avg(air_array_size, air_temp_array);
            air_humidity_avg = calc_avg(air_array_size, air_humidity_array);
            air_temp_cursor = 0;  // reset cursor
            Serial.print("Updated Air Temp Avg: "); Serial.println(air_temp_avg);
            Serial.print("Updated Air Humidity Avg: "); Serial.println(air_humidity_avg);
            return true;  // avg has been updated
        }
    }
    return false;  // avg has not been updated
}

/*******************************************************************
# WATER PH
*******************************************************************/

bool read_water_ph() {
    unsigned long cur_millis = millis();

    if (ph_read_prev_millis < cur_millis - ph_reading_delay) {
        ph_last = analogRead(pin_water_ph);
        ph_read_prev_millis = cur_millis;
        return true;
    }
    return false;
}

float convert_ph(float ph_raw) {
    float ph = (float)ph_raw * board_voltage / 1024 / 10 ;      // Convert the analog into millivolt
    ph = 3.5 * ph;                                              // Convert the millivolt into pH
    return ph;
}

bool ph_update() {
    if (read_water_ph() == true) {
        ph_array[ph_cursor] = ph_last;
        ph_cursor++;  // increment cursor
        if (ph_cursor >= ph_array_size) {
            float raw_avg = calc_avg(ph_array_size, ph_array);
            ph_avg = convert_ph(raw_avg);
            ph_cursor = 0;  // reset cursor
            Serial.print("Updated pH Avg: "); Serial.println(ph_avg);
            return true;  // avg has been updated
        }
    }
    return false;  // avg has not been updated
}

void set_ph() {
    float ph_hyst_min = ph_setpoint - ph_hyst;
    float ph_hyst_max = ph_setpoint + ph_hyst;

    if (ph_mode == 0) {  // idle mode
        if (ph_avg < ph_hyst_min) {  // ph below min limit
            ph_mode = 1;  // ph low mode
        }
        else if (ph_avg > ph_hyst_max) {  // ph above max limit
            ph_mode = 2;  // ph high mode
        }
        else {  // ph is within min and max limits
            digitalWrite(pin_ph_minus, LOW);  // turn off ph minus
            digitalWrite(pin_ph_plus, LOW);  // turn off ph plus
            if (ph_prev != 0) {
                Serial.println("Turning pH Plus and Minus Off");
            }
            ph_prev = 0;
        }
    }

    if (ph_mode == 1) {  // low mode
        if (ph_avg < ph_setpoint) {  // ph is below set point. Add ph plus
            unsigned long cur_millis = millis();
            if (cur_millis - ph_set_prev_millis > ph_pin_time) {
                ph_set_prev_millis = cur_millis;

                if (ph_pin_state == LOW) {
                    ph_pin_state = HIGH;
                    ph_pin_time = ph_pin_high_time;
                }
                else {
                    ph_pin_state = LOW;
                    ph_pin_time = ph_pin_low_time;
                }
                digitalWrite (pin_ph_plus, ph_pin_state);  // ph plus on with duty cycle
                digitalWrite (pin_ph_minus, LOW);  // ph minus off
                if (ph_prev != 1) {
                    Serial.println("Turning PH Plus On");
                }
                ph_prev = 1;
            }
        }
        else {  // ph_avg >= ph_setpoint
            ph_mode = 0;  // reset ph mode to idle mode
        }
    }

    if (ph_mode == 2) {  // high mode
        if (ph_avg > ph_setpoint) {
            unsigned long cur_millis = millis();
            if (cur_millis - ph_set_prev_millis > ph_pin_time) {
                ph_set_prev_millis = cur_millis;

                if (ph_pin_state == LOW) {
                    ph_pin_state = HIGH;
                    ph_pin_time = ph_pin_high_time;
                }
                else {
                    ph_pin_state = LOW;
                    ph_pin_time = ph_pin_low_time;
                }
                digitalWrite (pin_ph_minus, ph_pin_state);  // ph minus on with duty cycle
                digitalWrite (pin_ph_plus, LOW);  // ph plus off
                if (ph_prev != 2) {
                    Serial.println("Turning PH Minus On");
                }
                ph_prev = 2;
            }
        }
        else {  // ph_avg <= ph_setpoint
            ph_mode = 0;  // reset ph mode to idle mode
        }
    }
}

/*******************************************************************
# WATER TEMP
*******************************************************************/

bool read_water_temp() {
    unsigned long cur_millis = millis();

    if (water_prev_millis < cur_millis - water_reading_delay) {
        water_last = analogRead(pin_water_temp);
        water_prev_millis = cur_millis;
        return true;
    }
    return false;
}

bool water_temp_update() {
    if (read_water_temp() == true) {
        water_array[water_cursor] = water_last;
        water_cursor++;  // increment cursor
        if (water_cursor >= water_array_size) {
            water_avg = calc_avg(water_array_size, water_array);
            water_cursor = 0;  // reset cursor
            Serial.print("Updated Water Temp Avg: "); Serial.println(water_avg);
            return true;  // avg has been updated
        }
    }
    return false;  // avg has not been updated
}

/*******************************************************************
# LIGHT
*******************************************************************/

bool read_light() {
    unsigned long cur_millis = millis();

    if (light_prev_millis < cur_millis - light_reading_delay) {
        analogRead(pin_light);
        light_prev_millis = cur_millis;
        return true;
    }
    return false;
}

bool light_update() {
    if (read_light() == true) {
        light_array[light_cursor] = light_last;
        light_cursor++;  // increment cursor
        if (light_cursor >= light_array_size) {
            light_avg = calc_avg(light_array_size, light_array);
            light_cursor = 0;  // reset cursor
            Serial.print("Updated Light Avg: "); Serial.println(light_avg);
            return true;  // avg has been updated
        }
    }
    return false;  // avg has not been updated
}

/*******************************************************************
# FAN
*******************************************************************/

void set_fan() {
    if (air_temp_avg >= fan_high) {
        digitalWrite(pin_fan, HIGH);
        if (fan_prev == 0) {   // only print when state changes
            Serial.println("Turning Fan On");
        }
        fan_prev = 1;
    }
    else if (air_temp_avg <= (fan_high - fan_hyst)) {
        digitalWrite(pin_fan, LOW);
        if (fan_prev == 1) {   // only print when state changes
            Serial.println("Turning Fan Off");
        }
        fan_prev = 0;
    }
}

/*******************************************************************
# WATER LEVELS
*******************************************************************/

void water_level_update() {
    water_low = digitalRead(pin_water_low);
    water_med = digitalRead(pin_water_middle);
    water_high = digitalRead(pin_water_high);
}

void set_water_level() {
    if (water_low != water_low_norm_state) {
        if (water_med == water_med_norm_state || water_high == water_high_norm_state) {
            // TODO: Choose what to do in this state
            // Should never get to this state
        }
        else {
            water_filling = true;
        }
    }

    if (water_filling == true) {
        if (water_med == water_med_norm_state || water_high == water_high_norm_state) {
            digitalWrite(pin_solenoid, LOW);
            water_filling = false;
            if (solenoid_prev != 0) {
                Serial.println("Turning Solenoid Off");
            }
            solenoid_prev = 0;
        }
        else {
            digitalWrite(pin_solenoid, HIGH);
            if (solenoid_prev != 1) {
                Serial.println("Turning Solenoid On");
            }
            solenoid_prev = 1;
        }
    }
    else {
        digitalWrite(pin_solenoid, LOW);
        if (solenoid_prev != 0) {
                Serial.println("Turning Solenoid Off");
            }
            solenoid_prev = 0;
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
# SERIAL SEND
*******************************************************************/
void send_msg(String t, String m) {
    // Send payload
    String payload = t + MSG_SPLITTER + m + MSG_END;
    Serial1.print(payload);

    Serial.print("Sending message via Serial1: ");
    Serial.println(payload);
}

void heartbeat() {
    unsigned long cur_millis = millis();
    if (heartbeat_prev_millis < cur_millis - heartbeat_delay && cur_millis > heartbeat_delay) {
        heartbeat_count++;
        send_msg("heartbeat", String(heartbeat_count));
        heartbeat_prev_millis = cur_millis;
    }
}

void send_sensor_readings() {
    // Heartbeat
    if (send_all_readings) {
        send_msg("heartbeat", String(heartbeat_count));
    }
    // Relay Pins
    int ph_plus_relay = digitalRead(pin_ph_plus);
    int ph_minuis_relay = digitalRead(pin_ph_minus);
    int fan_relay = digitalRead(pin_fan);
    int solenoid_relay = digitalRead(pin_solenoid);
    if (ph_plus_relay != ph_plus_relay_prev || send_all_readings) {
        send_msg("ph_plus_relay", String(ph_plus_relay));
        ph_plus_relay_prev = ph_plus_relay;
    }
    if (ph_minuis_relay != ph_minuis_relay_prev || send_all_readings) {
        send_msg("ph_minuis_relay", String(ph_minuis_relay));
        ph_minuis_relay_prev = ph_minuis_relay;
    }
    if (fan_relay != fan_relay_prev || send_all_readings) {
        send_msg("fan_relay", String(fan_relay));
        fan_relay_prev = fan_relay;
    }
    if (solenoid_relay != solenoid_relay_prev || send_all_readings) {
        send_msg("solenoid_relay", String(solenoid_relay));
        solenoid_relay_prev = solenoid_relay;
    }
    // Test Mode
    if (test_mode != test_mode_prev || send_all_readings) {
        send_msg("test_mode", String(test_mode));
        test_mode_prev = test_mode;
    }
    // Manual Mode
    if (manual_mode != manual_mode_prev || send_all_readings) {
        send_msg("manual_mode", String(manual_mode));
        manual_mode_prev = manual_mode;
    }
    if (manual_ph_plus != manual_ph_plus_prev || send_all_readings) {
        send_msg("manual_ph_plus", String(manual_ph_plus));
        manual_ph_plus_prev = manual_ph_plus;
    }
    if (manual_ph_minus != manual_ph_minus_prev || send_all_readings) {
        send_msg("manual_ph_minus", String(manual_ph_minus));
        manual_ph_minus_prev = manual_ph_minus;
    }
    if (manual_fan != manual_fan_prev || send_all_readings) {
        send_msg("manual_fan", String(manual_fan));
        manual_fan_prev = manual_fan;
    }
    if (manual_solenoid != manual_solenoid_prev || send_all_readings) {
        send_msg("manual_solenoid", String(manual_solenoid));
        manual_solenoid_prev = manual_solenoid;
    }
    // Air Temp
    if (air_temp_avg != air_temp_avg_prev || send_all_readings) {
        send_msg("air_temp_avg", String(air_temp_avg));
        air_temp_avg_prev = air_temp_avg;
    }
    // Air Humidity
    if (air_humidity_avg != air_humidity_avg_prev || send_all_readings) {
        send_msg("air_humidity_avg", String(air_humidity_avg));
        air_humidity_avg_prev = air_humidity_avg;
    }
    // Water Temp
    if (water_avg != water_avg_prev || send_all_readings) {
        send_msg("water_avg", String(water_avg));
        water_avg_prev = water_avg;
    }
    // PH
    if (ph_avg != ph_avg_prev || send_all_readings) {
        send_msg("ph_avg", String(ph_avg));
        ph_avg_prev = ph_avg;
    }
    // PH Setpoint
    if (ph_setpoint != ph_setpoint_prev || send_all_readings) {
        send_msg("ph_setpoint", String(ph_setpoint));
        ph_setpoint_prev = ph_setpoint;
    }
    // PH Hyst Setpoint
    if (ph_hyst != ph_hyst_prev || send_all_readings) {
        send_msg("ph_hyst", String(ph_hyst));
        ph_hyst_prev = ph_hyst;
    }
    // PH Pin Low Time
    if (ph_pin_low_time != ph_pin_low_time_prev || send_all_readings) {
        send_msg("ph_pin_low_time", String(ph_pin_low_time));
        ph_pin_low_time_prev = ph_pin_low_time;
    }
    // PH Pin High Time
    if (ph_pin_high_time != ph_pin_high_time_prev || send_all_readings) {
        send_msg("ph_pin_high_time", String(ph_pin_high_time));
        ph_pin_high_time_prev = ph_pin_high_time;
    }
    // Light
    if (light_avg != light_avg_prev || send_all_readings) {
        send_msg("light_avg", String(light_avg));
        light_avg_prev = light_avg;
    }
    // Water Low Level
    if (water_low != water_low_prev || send_all_readings) {
        send_msg("water_low", String(water_low));
        water_low_prev = water_low;
    }
    // Water Medium Level
    if (water_med != water_med_prev || send_all_readings) {
        send_msg("water_med", String(water_med));
        water_med_prev = water_med;
    }
    // Water High Level
    if (water_high != water_high_prev || send_all_readings) {
        send_msg("water_high", String(water_high));
        water_high_prev = water_high;
    }
    // Fan Hyst Temp
    if (fan_hyst != fan_hyst_prev || send_all_readings) {
        send_msg("fan_hyst", String(fan_hyst));
        fan_hyst_prev = fan_hyst;
    }
    // Fan High Temp
    if (fan_high != fan_high_prev || send_all_readings) {
        send_msg("fan_high", String(fan_high));
        fan_high_prev = fan_high;
    }
    // heartbeat_delay
    if (heartbeat_delay != heartbeat_delay_prev || send_all_readings) {
        send_msg("heartbeat_delay", String(heartbeat_delay));
        heartbeat_delay_prev = heartbeat_delay;
    }
    // ph_reading_delay
    if (ph_reading_delay != ph_reading_delay_prev || send_all_readings) {
        send_msg("ph_reading_delay", String(ph_reading_delay));
        ph_reading_delay_prev = ph_reading_delay;
    }
    // water_reading_delay
    if (water_reading_delay != water_reading_delay_prev || send_all_readings) {
        send_msg("water_reading_delay", String(water_reading_delay));
        water_reading_delay_prev = water_reading_delay;
    }
    // air_reading_delay
    if (air_reading_delay != air_reading_delay_prev || send_all_readings) {
        send_msg("air_reading_delay", String(air_reading_delay));
        air_reading_delay_prev = air_reading_delay;
    }
    // light_reading_delay
    if (light_reading_delay != light_reading_delay_prev || send_all_readings) {
        send_msg("light_reading_delay", String(light_reading_delay));
        light_reading_delay_prev = light_reading_delay;
    }

    send_all_readings = false;
}

/*******************************************************************
# SERIAL READ
*******************************************************************/
void print_update() {
    Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
}

void test_mode_msg() {
    // PH
    if (topic == "ph_avg") {
        float x = msg.toFloat();
        ph_avg = x;
        print_update();
    }
    // Water Temp
    else if (topic == "water_avg") {
        float x = msg.toFloat();
        water_avg = x;
        print_update();
    }
    // Air Temp
    else if (topic == "air_temp_avg") {
        float x = msg.toFloat();
        air_temp_avg = x;
        print_update();
    }
    // Air Humidity
    else if (topic == "air_humidity_avg") {
        float x = msg.toFloat();
        air_humidity_avg = x;
        print_update();
    }
    // Light
    else if (topic == "light_avg") {
        float x = msg.toFloat();
        light_avg = x;
        print_update();
    }
    // Water Level
    else if (topic == "water_low") {
        if (msg == "0") {
            water_low = 0;
            print_update();
        }
        else if (msg == "1") {
            water_low = 1;
            print_update();
        }
    }
    else if (topic == "water_med") {
        if (msg == "0") {
            water_med = 0;
            print_update();
        }
        else if (msg == "1") {
            water_med = 1;
            print_update();
        }
    }
    else if (topic == "water_high") {
        if (msg == "0") {
            water_high = 0;
            print_update();
        }
        else if (msg == "1") {
            water_high = 1;
            print_update();
        }
    }
}

void process_msg() {
    
    if (topic == "send_all_readings") {
        if (msg == "1") {
            send_all_readings = true;
            print_update();
        }
    }
    // Test Mode
    else if (topic == "test_mode") {
        if (msg == "0") {
            test_mode = 0;
            print_update();
        }
        else if (msg == "1") {
            test_mode = 1;
            print_update();
        }
    }
    // Manual Mode
    else if (topic == "manual_mode") {
        if (msg == "0") {
            manual_mode = 0;
            print_update();
        }
        else if (msg == "1") {
            manual_mode = 1;
            print_update();
        }
    }
    else if (topic == "manual_ph_plus") {
        if (msg == "0") {
            manual_ph_plus = 0;
            print_update();
        }
        else if (msg == "1") {
            manual_ph_plus = 1;
            print_update();
        }
    }
    else if (topic == "manual_ph_minus") {
        if (msg == "0") {
            manual_ph_minus = 0;
            print_update();
        }
        else if (msg == "1") {
            manual_ph_minus = 1;
            print_update();
        }
    }
    else if (topic == "manual_fan") {
        if (msg == "0") {
            manual_fan = 0;
            print_update();
        }
        else if (msg == "1") {
            manual_fan = 1;
            print_update();
        }
    }
    else if (topic == "manual_solenoid") {
        if (msg == "0") {
            manual_solenoid = 0;
            print_update();
        }
        else if (msg == "1") {
            manual_solenoid = 1;
            print_update();
        }
    }
    // PH
    else if (topic == "ph_setpoint") {
        float x = msg.toFloat();
        ph_setpoint = x;
        print_update();
    }
    else if (topic == "ph_hyst") {
        float x = msg.toFloat();
        ph_hyst = x;
        print_update();
    }
    // Fan
    else if (topic == "fan_high") {
        float x = msg.toFloat();
        fan_high = x;
        print_update();
    }
    else if (topic == "fan_hyst") {
        float x = msg.toFloat();
        fan_hyst = x;
        print_update();
    }
    // Delays
    else if (topic == "heartbeat_delay") {
        unsigned long x = msg.toInt();
        heartbeat_delay = x;
        print_update();
    }
    else if (topic == "ph_reading_delay") {
        unsigned long x = msg.toInt();
        ph_reading_delay = x;
        print_update();
    }
    else if (topic == "ph_pin_low_time") {
        unsigned long x = msg.toInt();
        ph_pin_low_time = x;
        print_update();
    }
    else if (topic == "ph_pin_high_time") {
        unsigned long x = msg.toInt();
        ph_pin_high_time = x;
        print_update();
    }
    else if (topic == "water_reading_delay") {
        unsigned long x = msg.toInt();
        water_reading_delay = x;
        print_update();
    }
    else if (topic == "air_reading_delay") {
        unsigned long x = msg.toInt();
        air_reading_delay = x;
        print_update();
    }
    else if (topic == "light_reading_delay") {
        unsigned long x = msg.toInt();
        light_reading_delay = x;
        print_update();
    }
    else if (test_mode) {
        test_mode_msg();
    }
    
    // reset the topic and msg variables
    topic = "";
    msg = "";
}

/*
    SerialEvent occurs whenever a new data comes in the hardware serial RX. This
    routine is run between each time loop() runs, so using delay inside loop can
    delay response. Multiple bytes of data may be available.
    */
void serialEvent() {
    
    while (Serial1.available()) {
      char inChar = (char)Serial1.read();    // get the new byte

        if (inChar == MSG_SPLITTER) {
            topic = "";
            while (!char_queue.isEmpty ()) {
                topic += char_queue.dequeue();
            }
            if (topic.length() > 0) {
                Serial.print("Received topic: ");  Serial.println(topic);
            }
        }
        else if (inChar == MSG_END) {
            msg = "";
            while (!char_queue.isEmpty ()) {
                msg += char_queue.dequeue();
            }
            if (msg.length() > 0) {
                Serial.print("Received message: ");  Serial.println(msg);
                if (topic.length() > 0) {
                    process_msg();
                }
            }
        }
        else {
            char_queue.enqueue(inChar);  // add recieved to the char_queue
        }
    }
}

/*******************************************************************
# SETUP AND LOOP
*******************************************************************/

void setup() {
    pin_setup();                    // Setup pins as inputs and outputs
    Serial.begin(BAUD_RATE);        // Start Serial
    Serial1.begin(BAUD_RATE1);      // Start Serial1
    Serial.println("Serial Started");
    send_msg("alive", "1");         // Send message to clear buffer. First message doesn't go through for some reason?
    send_sensor_readings();         // Init sensor read values
    dht.begin();                    // Air Temp
    watch_dog_timer_setup();
    delay(100);
  }
  
  
void loop() {
    if (manual_mode == 0) {
        air_temp_update();
        set_fan();

        // ph_update();
        set_ph();

        // water_level_update();
        set_water_level();

        // water_temp_update();
        // light_update();
    }
    else {
        set_manual_mode();      // Manual mode function
    }
    send_sensor_readings();     // Write sensor readings to serial
    serialEvent();              // Process serial received messages
    heartbeat();                // Send heartbeat
    wdt_reset();                // Reset watch dog timer

    delay(10);
}