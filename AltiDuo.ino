/*
  Model Rocket dual altimeter Ver 1.4
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
#include <EEPROM.h>
#include <Adafruit_BMP085.h>

#define MAJOR_VERSION 1
#define MINOR_VERSION 4

// Turn on Serial Console
#define DEBUG //=true

// Use Metric (meters) for altitude reporting
///#define METRIC_UNIT
#undef METRIC_UNIT

// Use altitude reporting format beeping for each digit
#define BEEP_DIGIT
//#undef BEEP_DIGIT

// Use tone command to generate beeps. Undefine if using led for reporting
#define USE_TONE
//#undef USE_TONE

#ifdef METRIC_UNIT
#define FEET_IN_METER 1.0
#else
#define FEET_IN_METER 3.28084
#endif

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
boolean apogeeSaved = false;
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
  
  //init Kalman filter
  KalmanInit();
  
#ifdef DEBUG
  // initialize the serial communication for debuging
  Serial.begin(9600);
#endif
  
  Wire.begin();
  //Presure Sensor Initialisation
  bmp.begin();
  //our drogue has not been fired
  apogeeHasFired=false;
  mainHasFired=false;
  
  //Initialise the output pin
  pinMode(pinApogee, OUTPUT);
  pinMode(pinMain, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);
  
  pinMode(pinAltitude1, INPUT);
  pinMode(pinAltitude2, INPUT);
  
  pinMode(pinApogeeContinuity, INPUT);
  pinMode(pinMainContinuity, INPUT);
  //Make sure that the output are turned off
  digitalWrite(pinApogee, LOW);
  digitalWrite(pinMain, LOW);
  digitalWrite(pinSpeaker, LOW);
  
  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION,MINOR_VERSION);
 
 
  // Read in the previous altitude
  int previousApogee = readPreviousApogee();
  if (previousApogee > 0 ) {

    previousApogee *= FEET_IN_METER;
   for( int repeat =0; repeat < 2; repeat++) {   
    #ifdef BEEP_DIGIT 
      beepAltitudeNew(previousApogee);
    #else
      beepAltitude(previousApogee);
    #endif

    delay(1000);
   }
  }
  
  //number of measures to do to detect Apogee
  measures = 10;
  
  //initialise the deployement altitude for the main 
  mainDeployAltitude = 100;

  // On the Alti duo when you close the jumper you set it to 1
  // val is the left jumper and val1 is the right jumper
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
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i=0; i<50; i++){
    KalmanCalc(bmp.readAltitude());
   }
  //let's read the lauch site altitude
  long sum = 0;
  long curr = 0;
  for (int i=0; i<10; i++){
    curr =KalmanCalc(bmp.readAltitude());
    sum += curr;
    delay(50); }
  initialAltitude = (sum / 10.0);
  
  lastAltitude = 0; 

  liftoffAltitude = 20;

}

void loop()
{
  //read current altitude
  currAltitude = (KalmanCalc(bmp.readAltitude())- initialAltitude);

  if (( currAltitude > liftoffAltitude) != true)
  {
    continuityCheck(pinApogeeContinuity,1);
    delay(500);
    continuityCheck(pinMainContinuity,2);
    delay(500);
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
    if(apogeeSaved == false ) {
      writePreviousApogee(apogeeAltitude); 
      apogeeSaved = true;
    }
    
    beginBeepSeq();
    
    #ifdef BEEP_DIGIT 
      beepAltitudeNew(apogeeAltitude * FEET_IN_METER);
    #else
      beepAltitude(apogeeAltitude * FEET_IN_METER);
    #endif

    delay(1000);
  }
}

void continuityCheck(int pin, int beeps)
{
    #ifdef DEBUG
    Serial.print("continuity check pin:" );
    Serial.print(pin);
    Serial.print(" beeps:");
    Serial.println(beeps);
    #endif

  int val = 0;     // variable to store the read value
  // read the input pin to check the continuity if apogee has not fired
  if (apogeeHasFired == false )
  {
    val = digitalRead(pin);
    #ifdef DEBUG
    Serial.print("   value:" );
    Serial.println(val);
    #endif
    if (val == 0)
    {
      longBeepRepeat(beeps);
    }
    else
    {
      shortBeepRepeat(beeps);
    }
  }
}


///////////////////////////////////////////////////////////////
// fonction to beep the altitude in feet or meter
//
///////////////////////////////////////////////////////////
void beepAltitude(long altitude)
{
  int nbrLongBeep=0;
  int nbrShortBeep=0;
  int i;
  
  // this is the last thing that I need to write, some code to beep the altitude
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
  if (nbrLongBeep > 0) {
    shortBeepRepeat( nbrLongBeep );
  }

  if (nbrShortBeep > 0) {
    shortBeepRepeat( nbrShortBeep );
  }
  
  delay(5000);

}

void longBeepRepeat( int digit ) {
    #ifdef DEBUG
    Serial.println("longBeepRepeat: " );
    Serial.println( digit);
    #endif
  int i;
  for( i=0; i< digit; i++ ) {
    if( i > 0 ) {
      delay(250);
    }
    longBeep();
  }
}

void shortBeepRepeat( int digit ) {
    #ifdef DEBUG
    Serial.println("shortBeepRepeat: " );
    Serial.println( digit);
    #endif
  int i;
  for( i=0; i< digit; i++ ) {
    if( i > 0 ) {
      delay(250);
    }
    shortBeep();
  }
}

void beginBeepSeq()
{
  int i=0;
  #ifdef USE_TONE
    for (i=0; i<10;i++)
    {
      tone(pinSpeaker, 1600,1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  #endif
}
void longBeep()
{
    #ifdef DEBUG
    Serial.println("<<longBeep>>" );
    #endif
  #ifdef USE_TONE
    tone(pinSpeaker, 600,1000);
    delay(1000);
    noTone(pinSpeaker);
  #else
    digitalWrite(pinSpeaker, 1); // Toggle on/off
    delay(1000);
    digitalWrite(pinSpeaker, 0);
  #endif
}
void shortBeep()
{
    #ifdef DEBUG
    Serial.println("<<shortBeep>>" );
    #endif
  #ifdef USE_TONE
    tone(pinSpeaker, 600,250);
    delay(250);
    noTone(pinSpeaker);
  #else
    digitalWrite(pinSpeaker, 1);
    delay(250);
    digitalWrite(pinSpeaker, 0)
  #endif
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

void beepAltiVersion (int majorNbr, int minorNbr)
{
    #ifdef DEBUG
    Serial.println("Major Version:");
    Serial.println( majorNbr);
    Serial.println("Minor Version:");
    Serial.println( minorNbr);
    #endif

  longBeepRepeat(majorNbr);
  delay(250);
  shortBeepRepeat(minorNbr);
  delay(1000);
}

/*************************************************************************

 The following is some Code written by Leo Nutz and modified so that it works
  Output the maximum achieved altitude (apogee) via flashing LED / Buzzer

 *************************************************************************/
void beepAltitudeNew(uint32_t value)
{
  char Apogee_String[5];                        // Create an array with a buffer of 5 digits

  ultoa(value, Apogee_String, 10);              // Convert unsigned long to string array, radix 10 for decimal
  uint8_t length = strlen(Apogee_String);       // Get string length

  delay(3000);                                  // Pause for 3 seconds

  for(uint8_t i = 0; i < length; i++ )
  {
    delay(1000);                                // Pause 1 second for every digit output

    uint8_t digit = (Apogee_String[i] - '0');   // Convert ASCI to actual numerical digit
    if ( digit == 0 ) {
      digit = 10;
    }
    if ( digit == 0 ) {
      longBeepRepeat(1);
    } else {
      shortBeepRepeat(digit);
    }
  }
}

#define CONFIG_START 32

struct ConfigStruct {
  char app[7];
  int  majorVersion;
  int  minorVersion;
  int  altitude;
  int  cksum;  
};

int readPreviousApogee() {
  
  ConfigStruct config;
  
  int i;
  for( i=0; i< sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }
  // Verify:
  if ( strcmp( "AltDuo", config.app ) != 0 ) {
    return -1;
  }
  if ( config.majorVersion != MAJOR_VERSION ) {
    return -1;
  }
  if (config.minorVersion != MINOR_VERSION ) {
    return -1;
  }
  if ( config.cksum != 0xBA ) {
    return -1;
  }
  
  return config.altitude;
  
}

int writePreviousApogee( int altitude ) {
 ConfigStruct config;
 config.app[0] = 'A';
 config.app[1] = 'l';
 config.app[2] = 't';
 config.app[3] = 'D';
 config.app[4] = 'u';
 config.app[5] = 'o';
 config.app[6] = 0;
 config.majorVersion = MAJOR_VERSION;
 config.minorVersion = MINOR_VERSION;
 config.cksum = 0xBA;
 config.altitude = altitude;
 
 int i;
 for( i=0; i<sizeof(config); i++ ) {
   EEPROM.write(CONFIG_START+i, *((char*)&config + i));
 }
}

