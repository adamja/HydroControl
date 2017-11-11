/*
Program: Wemos Hydro Control
Version: 1.0.0
Author: Adam Jacob
Comments:

TODO:
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>     // Over the air 
#include <PubSubClient.h>   // MQTT library
#include <QueueArray.h>     // Queue library for holding serial input messages
#include <SoftwareSerial.h>

/* Serial Settings */
#define BAUD_RATE 9600              // Hardware Serial Baud Rate
#define BAUD_RATE_SS 115200         // Software Serial Baud Rate. Do not use 9600, it causes the wemos to crash and restart during transmission. 115200 appears to work well.
#define BUFSIZE 100
const char MSG_SPLITTER = '|';      // Splitter to seperate topic and message within a serial message
const char MSG_END = '$';           // Symbol to determine end of serial message
QueueArray <char> char_queue;       // Serial char buffer
String topic = "";                  
String msg = "";

/* Software Serial */
int pinRx = 14;   // D5
int pinTx = 12;   // D6
SoftwareSerial softSerial(pinRx, pinTx, false, BUFSIZE);  // SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, unsigned int buffSize = 64);

/* WiFi Settings */
const char* ssid     = "";      // WIFI SSID
const char* password = "";      // WIFI Password

/* MQTT */
const char* mqttSubTopic = "hydrocontroller/send/#";  // MQTT topic
String mqttPubTopic = "hydrocontroller/rec/";         // MQTT topic send
IPAddress broker(192,168,40,101);                     // Address of the MQTT broker  
#define CLIENT_ID "hydrocontroller"                   // Client ID to send to the broker
#define mqttUser ""                                   // broker username
#define mqttPass ""                                   // broker password
#define brokerPort 1883                               // broker port

/* WIFI */
WiFiClient wificlient;
PubSubClient client(wificlient);

void sendToSerial(String t, String m) {
  // Send payload to ardino via serial link
  // Capture the last section of the topic
  String temp = "";
  for (int i = 0; i < t.length(); i++) {
    if (t[i] == '/') {
      temp = "";
    }
    else {
      temp += t[i];
    }
  }
  // Create a message to send via serial on the format of: "topic|message$"
  String flush_payload = String(MSG_SPLITTER) + String(MSG_END);
  String payload = temp + MSG_SPLITTER + m + MSG_END;

  Serial.print("Sending via Software Serial: ");  Serial.println(payload);
  softSerial.print(flush_payload);    // Clear the buffer at the reciever prior to sending the message. Was having issues with junk readings at the receiver
  softSerial.print(payload);          // Send payload
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String msg = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg.concat((char)payload[i]);
  }
  Serial.println();
  sendToSerial(topic, msg);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CLIENT_ID, mqttUser, mqttPass)) {
      Serial.println("connected");
      client.subscribe(mqttSubTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendToMqtt() {
  String t = mqttPubTopic + topic;
  char charBufTopic[BUFSIZE];
  char charBufMsg[BUFSIZE];
  t.toCharArray(charBufTopic, BUFSIZE);  // Convert Topic String to char array
  msg.toCharArray(charBufMsg, BUFSIZE);  // Convert Message String to char array
  
  client.publish(charBufTopic, charBufMsg); // Publish to MQTT Broker

  topic = "";
  msg = "";
}

void serialEvent() {
  while (softSerial.available()) {
      char inChar = (char)softSerial.read();    // get the new byte

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
                    sendToMqtt();
                }
            }
        }
        else {
            char_queue.enqueue(inChar);  // add recieved to the char_queue
        }
    }
}

void setup() {
  Serial.begin(BAUD_RATE);
  softSerial.begin(BAUD_RATE_SS);
  
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("WiFi begun");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /* Prepare MQTT client */
  client.setServer(broker, brokerPort);
  client.setCallback(callback);
}

void loop() {
  ArduinoOTA.handle();
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, password);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
  }
  
  if (client.connected())
  {
    client.loop();
  }

  serialEvent();
  delay(10);
}