/*
  Model Rocket dual altimeter Ver 1.1
 Copyright Boris du Reau 2012-2013
 
 This is using a BMP085 presure sensor and an Atmega 328
 The following should fire the main at apogee if it is at least 50m above ground of the launch site
 and fire the main 100m before landing
 The Arduino board that I am using to load the program is an Arduino UNO ATmega328P 
 
 For the BMP085 pressure sensor
 Connect VCC of the BMP085 sensor to 5.0V! make sure that you are using the 5V sensor (GY-65 model)
 Connect GND to Ground
 Connect SCL to i2c clock - on 328 Arduino Uno/Duemilanove/etc thats Analog 5
 Connect SDA to i2c data - on 328 Arduino Uno/Duemilanove/etc thats Analog 4
 EOC is not used, it signifies an end of conversion
 XCLR is a reset pin, also not used here
 The micro swiches are connected on pin D6 and D7
 The main is connected to pin D8
 The apogee is connected to pin D9
 The main continuity test is connected to pin D10
 The apogee continuity test is connected to pin D11
 The speaker/buzzer is connected to pin D12
 */

//////////////////////////////////////////////////////////////////////////////
//
//  This is beta, I have not fligh it yet. It could cause premature ejections.
//  Damage your rocket, cause injuries etc...
//  Do not flash your altimeter unless you agree to contribute to the testing
//  and you do understand the risks.
//
/////////////////////////////////////////////////////////////////////////////
#include <Wire.h>

#include <Adafruit_BMP085.h>

#define DEBUG //=true

Adafruit_BMP085 bmp;



//ground level altitude
long initialAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
long liftoffAltitude;
long lastAltitude;
//Our drogue has been ejected i.e: apogee has been detected
boolean apogeeHasFired =false;
boolean mainHasFired=false;
//nbr of measures to do so that we are sure that apogee has been reached 
unsigned long measures;
unsigned long mainDeployAltitude;

const int pinAltitude1 = 8;
const int pinAltitude2 = 7;

const int pinApogee = 9;
const int pinMain = 13;
const int pinApogeeContinuity = 10;
const int pinMainContinuity = 11;
const int pinSpeaker = 12;

// Those are the pins for the ATMega 328
int nbrLongBeep=0;
int nbrShortBeep=0;

boolean NoBeep=false;

//Kalman Variables
float f_1=1.00000;  //cast as float
float kalman_x;
float kalman_x_last;
float kalman_p;
float kalman_p_last;
float kalman_k;
float kalman_q;
float kalman_r;
float kalman_x_temp;
float kalman_p_temp;
//end of Kalman Variables

void setup()
{
  int val = 0;     // variable to store the read value
  int val1 = 0;     // variable to store the read value
  
  //set up pins
  pinMode(pinApogeeContinuity, INPUT);
  pinMode(pinMainContinuity, INPUT);
  //Initialise the output pin
  pinMode(pinApogee, OUTPUT);
  pinMode(pinMain, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);
  
  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);
  //Make sure that the output are turned off
  digitalWrite(pinApogee, LOW);
  digitalWrite(pinMain, LOW);
  digitalWrite(pinSpeaker, LOW);
  
  //init Kalman filter
  KalmanInit();
  
  // initialize the serial communication for debuging
  Serial.begin(9600);

  Wire.begin();
  //Presure Sensor Initialisation
  bmp.begin();
  //initialisation
  beginBeepSeq();
  //our drogue has not been fired
  apogeeHasFired=false;
  mainHasFired=false;
  
  //let's read the lauch site altitude
  long sum = 0;
  for (int i=0; i<10; i++){
    sum += KalmanCalc(bmp.readAltitude());
    delay(500); }
  initialAltitude = (sum / 10.0);
  
  lastAltitude = initialAltitude; 
  liftoffAltitude = initialAltitude + 20;
  //number of measures to do to detect Apogee
  measures = 10;

  //initialise the deployement altitude for the main 
  mainDeployAltitude = 100;


  val = digitalRead(pinAltitude1); 
  val1 = digitalRead(pinAltitude2);  
  if(val == 0 && val1 ==0)
  {
    mainDeployAltitude = 50;
  }
  if(val == 0 && val1 ==1)
  {
    mainDeployAltitude = 100;
  }
  if(val == 1 && val1 ==0)
  {
    mainDeployAltitude = 150;
  }
  if(val == 1 && val1 ==1)
  {
    mainDeployAltitude = 200;
  }

}

void loop()
{
  //read current altitude
  currAltitude = (KalmanCalc(bmp.readAltitude())- initialAltitude);
  if (( currAltitude > liftoffAltitude) != true)
  {
    continuityCheck(pinApogeeContinuity);
    continuityCheck(pinMainContinuity);
  }
  
  //detect apogee
  if(currAltitude > liftoffAltitude)
  {

    if (currAltitude < lastAltitude)
    {
      measures = measures - 1;
      if (measures == 0)
      {
        //fire drogue
        digitalWrite(pinApogee, HIGH);
        delay (2000);
        apogeeHasFired=true;
        digitalWrite(pinApogee, LOW);
        apogeeAltitude = currAltitude;
      }  
    }
    else 
    {
      lastAltitude = currAltitude;
      measures = 10;
    } 
  }

 
  if ((currAltitude  < mainDeployAltitude) && apogeeHasFired == true && mainHasFired==false)
  {
    // Deploy main chute  100m before landing...
    digitalWrite(pinMain, HIGH);
    delay(2000);
    mainHasFired=true;
    mainAltitude= currAltitude;
    digitalWrite(pinMain, LOW);
  }
  if(apogeeHasFired == true && mainHasFired==true)
  {
    beginBeepSeq();
    
    //#ifdef DEBUG
    Serial.println("Initial Altitude:");
    Serial.println( initialAltitude);
    Serial.println("Apogee Altitude:");
    Serial.println( apogeeAltitude);
    beepAltitude(apogeeAltitude);
    //#endif
    beginBeepSeq();
    
    //#ifdef DEBUG
    Serial.println("Main Altitude:");
    Serial.println(mainAltitude);
    beepAltitude(mainAltitude);
    //#endif
  }
}

void continuityCheck(int pin)
{
  int val = 0;     // variable to store the read value
  // read the input pin to check the continuity if apogee has not fired
  if (apogeeHasFired == false )
  {
    val = digitalRead(pin);   
    if (val == 0)
    {
      //no continuity long beep
      longBeep();
    }
    else
    {
      //continuity short beep
      shortBeep();
    }
  }
}



void beepAltitude(long altitude)
{
  int i;
  // this is the laste thing that I need to write, some code to beep the altitude
  //altitude is in meters
  //find how many digits
  if(altitude > 99)
  {
    // 1 long beep per hundred meter
    nbrLongBeep= int(altitude /100);
    //then calculate the number of short beep
    nbrShortBeep = (altitude - (nbrLongBeep * 100)) / 10;
  } 
  else
  {
    nbrLongBeep = 0;
    nbrShortBeep = (altitude/10); 
  }
  Serial.println("long beep:");
  Serial.println( nbrLongBeep);
  if (nbrLongBeep > 0)
  for (i = 1; i <  nbrLongBeep +1 ; i++)
  {
    longBeep();
    delay(50);
  } 
 Serial.println("Short beep:" );
Serial.println(nbrShortBeep);
  if (nbrShortBeep > 0)
  for (i = 1; i <  nbrShortBeep +1 ; i++)
  {
    shortBeep();
    delay(50);
  } 

  delay(5000);

}

void beginBeepSeq()
{
  int i=0;
  if (NoBeep == false)
  {
    for (i=0; i<10;i++)
    {
      tone(pinSpeaker, 1600,1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
}
void longBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, 600,1000);
    delay(1500);
    noTone(pinSpeaker);
  }
}
void shortBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, 600,25);
    delay(300);
    noTone(pinSpeaker);
  }
}

//================================================================
// Kalman functions in your code
//================================================================

//Call KalmanInit() once.  

//KalmanInit() - Call before any iterations of KalmanCalc()
void KalmanInit()
{
   kalman_q=4.0001;  //filter parameters, you can play around with them
   kalman_r=.20001;  // but these values appear to be fairly optimal

   kalman_x = 0;
   kalman_p = 0;
   kalman_x_temp = 0;
   kalman_p_temp = 0;
   
   kalman_x_last = 0;
   kalman_p_last = 0;
   
}

//KalmanCalc() - Calculates new Kalman values from float value "altitude"
// This will be the ASL altitude during the flight, and the AGL altitude during dumps
float KalmanCalc (float altitude)
{
   
   //Predict kalman_x_temp, kalman_p_temp
   kalman_x_temp = kalman_x_last;
   kalman_p_temp = kalman_p_last + kalman_r;
   
   //Update kalman values
   kalman_k = (f_1/(kalman_p_temp + kalman_q)) * kalman_p_temp;
   kalman_x = kalman_x_temp + (kalman_k * (altitude - kalman_x_temp));
   kalman_p = (f_1 - kalman_k) * kalman_p_temp;
   
   //Save this state for next time
   kalman_x_last = kalman_x;
   kalman_p_last = kalman_p;
   
   //Assign current Kalman filtered altitude to working variables
   //KAlt = kalman_x; //FLOAT Kalman-filtered altitude value
  return kalman_x;
}  
