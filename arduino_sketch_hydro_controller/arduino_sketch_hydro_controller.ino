
float powervoltage=5;//define the power supply voltage.
#include <Wire.h>                  //One Wire library
#include <DHT.h>                   //DHT (Temperature/Humidity) library
#
int dht_dpin = 50;                //pin for DHT22
int pHPin = A2;                    //pin for pH probe
int pHPlusPin = 45;                //pin for Base pump (relay)
int pHMinPin = 43;                 //pin for Acid pump (relay)
int ventilatorPin = 49;            //pin for Fan (relay)
int floatLowPin = 8;               //pin for lower float sensor
int floatHighPin = 7;              //pin for upper float sensor
int solenoidPin = 47;              //pin for Solenoid valve (relay)
int lightSensor = A1;              //pin for Photoresistor
int growLights = 48;               //pin for Grow Lights (relay)


//*********Declaring Variables************************************//
int tankProgState = 1;             //returns the state of tank program - on or off
float pH;                          //generates the value of pH
boolean smoothPh = 1;              //variable that sets smoothing of pH on or off
float Offset = 0.00;               //deviation from true pH compensation (if necessary)
unsigned long int avgValue;        //stores the average value of the pH sensor
float b;
int buf[14],temp;

const int numReadings = 10;        
int readings[numReadings];         //the readings from the analog input
int index = 0;                     //the index of the current reading
int total = 0;                     //the running total
int average = 0;                   //the average

int count=0;

int ledState = LOW;                //variables for pulsing the pump
long previousMillis = 0;           //             |
long pinHighTime = 100;            //             |
long pinLowTime = 7500;            //             |
long pinTime = 100;                //             |

int sdState = LOW;                 //variables for delayed writing to SD card
long sdPreviousMillis = 0;         //             |
long sdTime = 7500;                //             |

int pmem = 0;                      //check which page your on
float Setpoint = 7.0               //holds value for Setpoint - pH?
float HysterisMin;                 //Minimum deviation from Setpoint
float HysterisPlus;                //Maximum deviation from Setpoint
float SetHysteris = 4.0            //Holds the value for Hysteris - pH?
float FanTemp;                     //Holds the set value for temperature
float FanHumid;                    //Holds the set value for humidity
float fanHysteris = 2;             //Set value for hysteris tuning Fan
float LightTime;                   //Holds the set value for amount of time plants should have light

int lightADCReading;               //variables for measuring the light
double currentLightInLux;          //              |
double lightInputVoltage;          //              |
double lightResistance;            //              |


#define DHTTYPE DHT22              //define which DHT chip is used - DHT11 or DHT22
byte bGlobalErr;                   //for passing error code back.
byte dht_dat[4];

void setup() 
{
// initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  smoothArraySetup();          //Sets the array for smoothing the pH value to 0
  logicSetup(); //Replaces the void Setup

}

void loop()
{

  logicLoop();                //pH algorithm loops through this one, also the smooting of the signal
  fotoLoop();                 //Light measurements loop through this one
  FanControl();               //Fans are controlled in this loop
  TankProgControl();          //Conrolling loop for refilling the tank
}

void InitDHT()
{
  pinMode(dht_dpin,OUTPUT);
  digitalWrite(dht_dpin,HIGH);
}

void smoothArraySetup()
{
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    readings[thisReading] = 0;
  }
}

void logicSetup()
{

  pinMode(pHPlusPin, OUTPUT);
  pinMode(pHMinPin, OUTPUT);
  pinMode(ventilatorPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);

  pmem==0;
  SetHysteris = 0;
  InitDHT();
  delay(300);
}

void ReadDHT()f
{
  bGlobalErr=0;
  byte dht_in;
  byte i;
  digitalWrite(dht_dpin,LOW);
  delay(23);
  digitalWrite(dht_dpin,HIGH);
  delayMicroseconds(40);
  pinMode(dht_dpin,INPUT);
  dht_in=digitalRead(dht_dpin);

  if(dht_in)
  {
    bGlobalErr=1;//dht start condition 1 not met
    return;
  }

  delayMicroseconds(80);
  dht_in=digitalRead(dht_dpin);

  if(!dht_in)
  {
    bGlobalErr=2;//dht start condition 2 not met
    return;
  }

  delayMicroseconds(80);
  for (i=0; i<5; i++)
    dht_dat[i] = read_dht_dat();

  pinMode(dht_dpin,OUTPUT);

  digitalWrite(dht_dpin,HIGH);

  byte dht_check_sum =
    dht_dat[0]+dht_dat[1]+dht_dat[2]+dht_dat[3];

  if(dht_dat[4]!= dht_check_sum)
  {
    bGlobalErr=3;
  }
}

byte read_dht_dat()
{
  byte i = 0;
  byte result=0;
  for(i=0; i< 8; i++)
  {
    while(digitalRead(dht_dpin)==LOW);
    delayMicroseconds(45);

    if (digitalRead(dht_dpin)==HIGH)
      result |=(1<<(7-i));

    while (digitalRead(dht_dpin)==HIGH);
  }
  return result;
}


void logicLoop()
{
  ReadDHT();                                        //call DHT chip for data
/*
  if (smoothPh == 0)                                //If smoothPh = 0 then no smooting is used
  {                                                 //                  |
    float sensorValue = 0;                          //Default the Value = 0
    sensorValue = analogRead(pHPin);                //sensorValue gets the value from the pH probe
    pH = (sensorValue * 5.0 / 1024 * 3.5 + Offset); //pH = the calculated value
                                                    //                  |
    HysterisMin = (Setpoint - SetHysteris);         //HysterisMin = the lowest value that is allowed by the program. Lower then this and a Base is added.
    HysterisPlus = (Setpoint + SetHysteris);        //HysterisPlus = the highest value that is allowed by the program. Higher and an acid is added.
                                                    //                  |
    if (pmem == 0)                                  //If pmem equals 0, then goto the next if statement.                 
    {                                               //                  |
      if (pH < HysterisMin)                         //If pH is smaller then HysterisMin, then set pmem to 1
      {                                             //                  |
        pmem = 1;                                   //                  |
      }                                             //                  |
                                                    //                  |
      if (pH >= HysterisMin && pH <= HysterisPlus)  //If pH is greater or the same as HysterisMin AND pH is smaller of the same as HysterisPlus, then do nothing.
      {                                             //                  |
        digitalWrite (pHPlusPin, LOW);              //Set base pump to off position
        digitalWrite (pHMinPin, LOW);               //Set acid pump to off position
      }                                             //                  |
                                                    //                  |
      if (pH > HysterisPlus)                        //If pH is greater then HysterisPlus, set pmem to 2
      {                                             //                  |
        pmem = 2;                                   //                  |
      }                                             //                  |
    }                                               //                  |
                                                    //                  |
                                                    //                  |
    if (pmem == 1)                                  //If pmem equals 1, the goto next if statement
    {                                               //                  |
      if (pH < HysterisMin)                         //If pH is smaller then HysterisMin, then 
      {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis > pinTime)
        {
          previousMillis = currentMillis;

          if (ledState == LOW)
          {
            ledState = HIGH;
            pinTime = pinHighTime;
          }
          else
          {
            ledState = LOW;
            pinTime = pinLowTime;
          }
          digitalWrite (pHPlusPin, ledState);
          digitalWrite (pHMinPin, LOW);
        }      
      }

      if (pH >= HysterisMin && pH < Setpoint)
      {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis > pinTime)
        {
          previousMillis = currentMillis;

          if (ledState == LOW)
          {
            ledState = HIGH;
            pinTime = pinHighTime;
          }
          else
          {
            ledState = LOW;
            pinTime = pinLowTime;
          }
          digitalWrite (pHPlusPin, ledState);
          digitalWrite (pHMinPin, LOW);
        }      
      }

      if (pH >= Setpoint)
      {
        pmem = 0;
      }
    }  

    if (pmem == 2)
    {
      if (pH > HysterisPlus)
      {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis > pinTime)
        {
          previousMillis = currentMillis;

          if (ledState == LOW)
          {
            ledState = HIGH;
            pinTime = pinHighTime;
          }
          else
          {
            ledState = LOW;
            pinTime = pinLowTime;
          }
          digitalWrite (pHMinPin, ledState);
          digitalWrite (pHPlusPin, LOW);
        }      
      }

      if (pH <= HysterisPlus && pH > Setpoint)
      {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis > pinTime)
        {
          previousMillis = currentMillis;

          if (ledState == LOW)
          {
            ledState = HIGH;
            pinTime = pinHighTime;
          }
          else
          {
            ledState = LOW;
            pinTime = pinLowTime;
          }
          digitalWrite (pHMinPin, ledState);
          digitalWrite (pHPlusPin, LOW);
        }      
      }

      if (pH <= Setpoint)
      {
        pmem = 0;
      }
    }  
  }
  */   
  if (smoothPh == 1)
  { 
    for(int i=0;i<14;i++)
    {
      buf[i]=analogRead(pHPin);
      delay(10);
    }
    for(int i=0;i<13;i++)                            //sort the analog values from small to large
    {
      for(int j=i+1;j<14;j++)
      {
        if(buf[i]>buf[j])
        {
          temp=buf[i];
          buf[i]=buf[j];
          buf[j]=temp;
        } 
      }
    }
    avgValue=0;
    for(int i=2;i<10;i++)                            //take the average value of 10 center sample
      avgValue+=buf[i];
    float phValue=(float)avgValue*5.0/1024/10;       //convert the analog into millivolt
    pH=3.5*phValue;                                 //convert the millivolt into pH

    HysterisMin = (Setpoint - SetHysteris);
    HysterisPlus = (Setpoint + SetHysteris);

    ++count;
    if (count > 10)
    {
      count = 10;
    }

    if (count == 10)
    {  
      if (pmem == 0)
      {
        if (pH < HysterisMin)
        {
          pmem = 1;  // pH low pump on
        }

        if (pH >= HysterisMin && pH <= HysterisPlus)
        {
          digitalWrite (pHPlusPin, LOW);  // pH high pump off
          digitalWrite (pHMinPin, LOW);   // pH low pump off
        }

        if (pH > HysterisPlus)
        {
          pmem = 2;  // pH high pump on
        }
      }


      if (pmem == 1)  // low bad point
      {
        if (pH < HysterisMin)
        {
          unsigned long currentMillis = millis();
          if(currentMillis - previousMillis > pinTime)
          {
            previousMillis = currentMillis;

            if (ledState == LOW)
            {
              ledState = HIGH;
              pinTime = pinHighTime;
            }
            else
            {
              ledState = LOW;
              pinTime = pinLowTime;
            }
            digitalWrite (pHPlusPin, ledState);
            digitalWrite (pHMinPin, LOW);
          }      
        }

        if (pH >= HysterisMin && pH < Setpoint)  // ph currently low - add ph
        {
          unsigned long currentMillis = millis();
          if(currentMillis - previousMillis > pinTime)
          {
            previousMillis = currentMillis;

            if (ledState == LOW)
            {
              ledState = HIGH;
              pinTime = pinHighTime;
            }
            else
            {
              ledState = LOW;
              pinTime = pinLowTime;
            }
            digitalWrite (pHPlusPin, ledState);
            digitalWrite (pHMinPin, LOW);
          }      
        }

        if (pH >= Setpoint)
        {
          pmem = 0;
        }
      }  

      if (pmem == 2)  // ph currently high - minus ph
      {
        if (pH > HysterisPlus)
        {
          unsigned long currentMillis = millis();
          if(currentMillis - previousMillis > pinTime)
          {
            previousMillis = currentMillis;

            if (ledState == LOW)
            {
              ledState = HIGH;
              pinTime = pinHighTime;
            }
            else
            {
              ledState = LOW;
              pinTime = pinLowTime;
            }
            digitalWrite (pHMinPin, ledState);
            digitalWrite (pHPlusPin, LOW);
          }      
        }

        if (pH <= HysterisPlus && pH > Setpoint)
        {
          unsigned long currentMillis = millis();
          if(currentMillis - previousMillis > pinTime)
          {
            previousMillis = currentMillis;

            if (ledState == LOW)
            {
              ledState = HIGH;
              pinTime = pinHighTime;
            }
            else
            {
              ledState = LOW;
              pinTime = pinLowTime;
            }
            digitalWrite (pHMinPin, ledState);
            digitalWrite (pHPlusPin, LOW);
          }      
        }

        if (pH <= Setpoint)
        {
          pmem = 0;
        }
      }  
    } 
  }
  delay(250);               
}

void fotoLoop()
{
  lightADCReading = analogRead(lightSensor);
  // Calculating the voltage of the ADC for light
  lightInputVoltage = 5.0 * ((double)lightADCReading / 1024.0);
  // Calculating the resistance of the photoresistor in the voltage divider
  lightResistance = (10.0 * 5.0) / lightInputVoltage - 10.0;
  // Calculating the intensity of light in lux       
  currentLightInLux = 255.84 * pow(lightResistance, -10/9);
}

void FanControl()
{
  if ((dht_dat[0] >= FanHumid + fanHysteris) && (dht_dat[2] >= FanTemp + fanHysteris) || (dht_dat[0] >= FanHumid + fanHysteris + 15) || (dht_dat[2] >= FanTemp + fanHysteris + 5))
  {
    digitalWrite(ventilatorPin, HIGH);
  }
  else if ((dht_dat[0] <= FanHumid - fanHysteris) && (dht_dat[2] <= FanTemp - fanHysteris) || (dht_dat[0] <= FanHumid - fanHysteris - 10) || (dht_dat[2] <= FanTemp - fanHysteris - 5))
  {
    digitalWrite(ventilatorPin, LOW);
  }
}

void TankProgControl()
{
  if (tankProgState == 0)
  {
  }
  if (tankProgState == 1)
  {
    int levelHigh = LOW;
    int levelLow = LOW;

    levelHigh = digitalRead(floatHighPin);
    levelLow = digitalRead(floatLowPin);

    if (levelHigh == LOW)
    {
      if (levelLow == LOW)
      {
        digitalWrite(solenoidPin, HIGH); //solenoid valve open.
      }
    }
    else
    {
      if (levelLow == HIGH)
      {
        digitalWrite(solenoidPin, LOW); //solenoid valve closed.
      }
    }
  }
}



