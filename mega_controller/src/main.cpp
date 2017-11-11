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
    - Add a timed message to send out that the controller is still alive
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
#define pin_ph_plus 4            // ph plus relay
#define pin_ph_minus 5           // ph minus relay
#define pin_fan 6                // Fan relay
#define pin_solenoid 7           // Water solenoid relay

/*******************************************************************
# VARIABLE SETUP                 
*******************************************************************/

// SERIAL
#define BAUD_RATE 9600              // Serial Baud Rate
#define BAUD_RATE1 115200             // Serial1 Baud Rate
const char MSG_SPLITTER = '|';      // Splitter to seperate topic and message within a serial message
const char MSG_END = '$';           // Symbol to determine end of serial message
QueueArray <char> char_queue;       // Serial char buffer
String topic = "";                  
String msg = "";

// Heartbear
unsigned long heartbeat_time = 10000;      // Send keep alive message at this interval
unsigned long heartbeat_prev_millis = 0;

// BOARD
float board_voltage = 5;        // Arduino Mega analogue input reads at 5V

// MANUAL VARIABLES
bool manual_override = false;   // Automation stops when set to true
int manual_ph_plus = 0;         // Default off
int manual_ph_minus = 0;        // Default off
int manual_fan = 0;             // Default off
int manual_solenoid = 0;        // Default off

// PH VARIABLES
int ph_mode = 0;  // 0 = idle | 1 = ph low | 2 = ph high
long prev_millis = 0;
float ph_setpoint = 7.0;
float ph_hyst_set = 2.0;
const int ph_array_size = 10;
int ph_cursor = 0;
float ph_array[ph_array_size];
float ph_last = 0.0;
float ph_avg = ph_setpoint;
int ph_pin_state = LOW;
unsigned long ph_pin_time = 100;
unsigned long ph_pin_low_time = 7500;
unsigned long ph_pin_high_time = 100;


// WATER TEMP VARIABLES
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

// LIGHT VARIABLES
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

float air_temp_avg_prev = air_temp_avg;
float air_humidity_avg_prev = air_humidity_avg;
float water_avg_prev = water_avg;
float ph_avg_prev = ph_avg;
float ph_setpoint_prev = ph_setpoint;
float ph_hyst_set_prev = ph_hyst_set;
unsigned long ph_pin_low_time_prev = ph_pin_low_time;
unsigned long ph_pin_high_time_prev = ph_pin_high_time;
float light_avg_prev = light_avg;
int water_low_prev = water_low;
int water_med_prev = water_med;
int water_high_prev = water_high;
float fan_hyst_prev = fan_hyst;
float fan_high_prev = fan_high;


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
    float h = dht.readHumidity();  // Read humidity
    float t = dht.readTemperature();  // Read temperature
  
    if (isnan(h) || isnan(t)) {
        return false;  // Failed to read, return false
    }
    air_temp_last = h;
    air_temp_last = t;
    return true;
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
    ph_last = analogRead(pin_water_ph);
    return true;
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
    float ph_hyst_min = ph_setpoint - ph_hyst_set;
    float ph_hyst_max = ph_setpoint + ph_hyst_set;

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
            if (cur_millis - prev_millis > ph_pin_time) {
                prev_millis = cur_millis;

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
            if (cur_millis - prev_millis > ph_pin_time) {
                prev_millis = cur_millis;

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
    String payload = t + MSG_SPLITTER + m + MSG_END;
    Serial1.print(payload);

    Serial.print("Sending message via Serial1: ");
    Serial.println(payload);
}

void send_sensor_readings() {
    // Air Temp
    if (air_temp_avg != air_temp_avg_prev) {
        send_msg("air_temp_avg", String(air_temp_avg));
        air_temp_avg_prev = air_temp_avg;
    }
    // Air Humidity
    if (air_humidity_avg != air_humidity_avg_prev) {
        send_msg("air_humidity_avg", String(air_humidity_avg));
        air_humidity_avg_prev = air_humidity_avg;
    }
    // Water Temp
    if (water_avg != water_avg_prev) {
        send_msg("water_avg", String(water_avg));
        water_avg_prev = water_avg;
    }
    // PH
    if (ph_avg != ph_avg_prev) {
        send_msg("ph_avg", String(ph_avg));
        ph_avg_prev = ph_avg;
    }
    // PH Setpoint
    if (ph_setpoint != ph_setpoint_prev) {
        send_msg("ph_setpoint", String(ph_setpoint));
        ph_setpoint_prev = ph_setpoint;
    }
    // PH Hyst Setpoint
    if (ph_hyst_set != ph_hyst_set_prev) {
        send_msg("ph_hyst_set", String(ph_hyst_set));
        ph_hyst_set_prev = ph_hyst_set;
    }
    // PH Pin Low Time
    if (ph_pin_low_time != ph_pin_low_time_prev) {
        send_msg("ph_pin_low_time", String(ph_pin_low_time));
        ph_pin_low_time_prev = ph_pin_low_time;
    }
    // PH Pin High Time
    if (ph_pin_high_time != ph_pin_high_time_prev) {
        send_msg("ph_pin_high_time", String(ph_pin_high_time));
        ph_pin_high_time_prev = ph_pin_high_time;
    }
    // Light
    if (light_avg != light_avg_prev) {
        send_msg("light_avg", String(light_avg));
        light_avg_prev = light_avg;
    }
    // Water Low Level
    if (water_low != water_low_prev) {
        send_msg("water_low", String(water_low));
        water_low_prev = water_low;
    }
    // Water Medium Level
    if (water_med != water_med_prev) {
        send_msg("water_med", String(water_med));
        water_med_prev = water_med;
    }
    // Water High Level
    if (water_high != water_high_prev) {
        send_msg("water_high", String(water_high));
        water_high_prev = water_high;
    }
    // Fan Hyst Temp
    if (fan_hyst != fan_hyst_prev) {
        send_msg("fan_hyst", String(fan_hyst));
        fan_hyst_prev = fan_hyst;
    }
    // Fan High Temp
    if (fan_high != fan_high_prev) {
        send_msg("fan_high", String(fan_high));
        fan_high_prev = fan_high;
    }
}

/*******************************************************************
# SERIAL READ
*******************************************************************/
void testing_msg() {
    // PH
    if (topic == "ph_setpoint") {
        float x = msg.toFloat();
        ph_setpoint = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    else if (topic == "ph_hyst_set") {
        float x = msg.toFloat();
        ph_hyst_set = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    else if (topic == "ph_avg") {
        float x = msg.toFloat();
        ph_avg = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    else if (topic == "ph_pin_low_time") {
        float x = msg.toInt();
        ph_pin_low_time = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    else if (topic == "ph_pin_high_time") {
        float x = msg.toInt();
        ph_pin_high_time = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    // Water Temp
    else if (topic == "water_avg") {
        float x = msg.toInt();
        water_avg = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }

    // Air Temp
    else if (topic == "air_temp_avg") {
        float x = msg.toInt();
        air_temp_avg = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    // Air Humidity
    else if (topic == "air_humidity_avg") {
        float x = msg.toInt();
        air_humidity_avg = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    // Light
    else if (topic == "light_avg") {
        float x = msg.toInt();
        light_avg = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }

    // Fan
    else if (topic == "fan_hyst") {
        float x = msg.toInt();
        fan_hyst = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    else if (topic == "fan_high") {
        float x = msg.toInt();
        fan_high = x;
        Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
    }
    // Water Level
    if (topic == "water_low") {
        if (msg == "0") {
            water_low = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            water_low = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "water_med") {
        if (msg == "0") {
            water_med = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            water_med = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "water_high") {
        if (msg == "0") {
            water_high = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            water_high = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
}

void process_msg() {
    if (topic == "manual_mode") {
        if (msg == "0") {
            manual_override = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            manual_override = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "set_ph_plus") {
        if (msg == "0") {
            manual_ph_plus = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            manual_ph_plus = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "set_ph_minus") {
        if (msg == "0") {
            manual_ph_minus = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            manual_ph_minus = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "set_fan") {
        if (msg == "0") {
            manual_fan = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            manual_fan = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    if (topic == "set_solenoid") {
        if (msg == "0") {
            manual_solenoid = 0;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
        else if (msg == "1") {
            manual_solenoid = 1;
            Serial.print("Updated "); Serial.print(topic); Serial.print(" to: "); Serial.println(msg);
        }
    }
    testing_msg();
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
    dht.begin();                  // Air Temp
  }
  
  
void loop() {
    if (manual_override == 0) {
        // if (air_temp_update()) {
        //     set_fan();
        // }
        // if (ph_update()) {
        //     ph_update()
        // }
        // if (water_level_update()) {
        //     set_water_level();
        // }
        water_temp_update();
        light_update();
        send_sensor_readings();  // Write sensor readings to serial
  
        set_ph();
        set_fan();
        set_water_level();
        
    }
    else {
        set_manual_mode();  // Manual mode function
    }
    serialEvent();  // Process serial received messages

    unsigned long cur_millis = millis();
    if (heartbeat_prev_millis < cur_millis - heartbeat_time && cur_millis > heartbeat_time) {
        send_msg("heartbeat", "heartbeat");
        heartbeat_prev_millis = cur_millis;
    }

    delay(10);
}