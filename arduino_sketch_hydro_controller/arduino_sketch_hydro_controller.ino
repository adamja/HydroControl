// edit comment hello adam
float powervoltage=5;//define the power supply voltage.
#include <Wire.h>                  //One Wire library
#include <RTClib.h>                //Real Time Clock library
#include <DHT.h>                   //DHT (Temperature/Humidity) library

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

RTC_DS1307 RTC;

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

int lightADCReading;               //variables for measuring the light
double currentLightInLux;          //              |
double lightInputVoltage;          //              |
double lightResistance;            //              |

DateTime now;                      //call current Date and Time

#define DHTTYPE DHT22              //define which DHT chip is used - DHT11 or DHT22
byte bGlobalErr;                   //for passing error code back.
byte dht_dat[4];

void setup() 
{
// initialize serial communication at 9600 bits per second:
  smoothArraySetup();          //Sets the array for smoothing the pH value to 0
  logicsetup(); //Replaces the void Setup
  timeSetup();   
  start_time = now.get(); //initialises the RTC module
  seconds_elapsed_total = 0;

}

void loop()
{

  logicLoop();                //pH algorithm loops through this one, also the smooting of the signal
  fotoLoop();                 //Light measurements loop through this one
  FanControl();               //Fans are controlled in this loop
  TankProgControl();          //Conrolling loop for refilling the tank
  LightControl();             //Controls when the grow lights are turned on
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

  InitDHT();
  delay(300);
}

void logicLoop()
{
  ReadDHT();                                        //call DHT chip for data

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
  if (smoothPh == 1)
  {
   // total = total - readings[index];
   // readings[index] = analogRead(pHPin);
   // total = total + readings[index];
   // index = index + 1;

   // if (index >= numReadings)
   // {
    //  index = 0;
   // }

  //  average = total / numReadings;

    //float sensorValue = 0;
    //sensorValue = average;
    //pH = (0.0178 * sensorValue - 1.889);
    
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
          pmem = 1;
        }

        if (pH >= HysterisMin && pH <= HysterisPlus)
        {
          digitalWrite (pHPlusPin, LOW);
          digitalWrite (pHMinPin, LOW);
        }

        if (pH > HysterisPlus)
        {
          pmem = 2;
        }
      }


      if (pmem == 1)
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
  }


  if (page == 0)
  {
    myGLCD.printNumF(pH, 2, 91, 23);
    myGLCD.printNumI(dht_dat[0], 91, 115, 3);
    myGLCD.printNumI(dht_dat[2], 91, 69, 3);
    myGLCD.printNumI(currentLightInLux, 91, 162, 4);
  }
  delay(250);               
}




void loop() {
float temperature1;
float ph;
// read the input on analog pin 0:
float sensorValue_ph = analogRead(A2);
float sensorValue_temp = analogRead(A0);
// print out the value you read:
temperature1=(sensorValue_temp/1023)*powervoltage*100;
ph=0.0178*sensorValue_ph-2.9;
//Serial.print("The room temperature degree is:");
Serial.println(temperature1,1);
//Serial.println(temperature2,1)
Serial.println(ph,2);
if (temperature1>27.0)
{
  analogWrite(motorpin, 100);
}

if (temperature<26.0)
{
  analogWrite(motorpin, 0);
}
delay(10000); // delay in between reads for stability
}
