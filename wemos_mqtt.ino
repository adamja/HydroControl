#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>


char buffer[10];

/* PIN SETUP */
#define relayPin D1

/* WiFi Settings */
const char* ssid     = "ssid";  // WIFI SSID
const char* password = "pass";    // WIFI Password

/* Serial */
String serialString = "";     // Flag to know if more data is to come
bool serialComplete = false;  // A string to hold incoming data

/* MQTT */
const char* mqttTopic = "home/wemos/01";      // MQTT topic
const char* mqttTopicSend = "home/wemos/01/send"; // MQTT topic send
IPAddress broker(192,168,40,100);             // Address of the MQTT broker
#define CLIENT_ID "id"                  // Client ID to send to the broker
#define mqttUser "user"                       // broker username
#define mqttPass "pass"                // broker password
#define brokerPort 1883                       // broker port

/* WIFI */
WiFiClient wificlient;
PubSubClient client(wificlient);


void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  
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

  serialIn();
  mqttOut();
  
  //ESP.deepSleep(10000);
  delay(100);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String s = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    s.concat((char)payload[i]);
  }
  Serial.println();
  checkPayload(s);
  forwardPayload(s);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CLIENT_ID, mqttUser, mqttPass)) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void serialIn() {
  // 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '$') {
      serialComplete = true;
      Serial.print("Reveived from serial: ");
      Serial.println(serialString);
    }
    else {
      serialString += inChar;
    }
  }
}

void serialOut(String s) {
  //
  Serial.print("Message send via serial: ");
  Serial.println(s);
}

void forwardPayload(String s) {
  // Send payload to ardino via serial link
  serialOut(s);
}

void mqttOut() {
  if(serialComplete) 
  {
    char charBuf[200];
    serialString.toCharArray(charBuf, 200);  // Convert String to char array
    client.publish(mqttTopicSend, charBuf);
    serialString = "";
    serialComplete = false;
  }
}

void checkPayload(String s) {
  if(s.equals("on")) {
    digitalWrite(relayPin, HIGH);
    Serial.println("Relay On");
  } else if(s.equals("off")) {
    digitalWrite(relayPin, LOW);
    Serial.println("Relay Off");
  }
}
