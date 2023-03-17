#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "MPU9250.h"
//#include "Adafruit_FONA.h"

#define FONA_RX 3
#define FONA_TX 4
#define FONA_RST 2

// The TinyGPS++ object
TinyGPSPlus gps;

//Create software serial object to communicate with SIM800L
SoftwareSerial ssGSM(3, 4); //SIM800L Tx & Rx is connected to Arduino #3 & #2
//SoftwareSerial ssGSM(11, 12); //blind gelegt, damit die Variable definiert ist.
SoftwareSerial ssGPS(6, 7); //GPS Tx & Rx is connected to Arduino #7 & #6
SoftwareSerial *fonaSerial = &ssGSM;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);

//Adafruit_FONA fona = Adafruit_FONA(FONA_RST);


int status;
int awake = 0;

unsigned long start = millis();
unsigned long rm2timer = millis();

// runmodes:
// 0 unarmed, waiting for SMS to arm
// 1 armed, waiting for gyro to alert or SMS to disarm
// 2 alert, reading GPS, after 9 Min going to runmode 3
// 3 send, sending last good GPS Position and return to runmode 2
// 99 debugging , only execute statements in block


int runmode = 0;


void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);

  Serial.print("Initializing");
  delay(100);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  //Begin serial communication with Arduino and GPS
  Serial.print(".");
  ssGPS.begin(4800);
  ssGPS.stopListening();
  //Begin serial communication with Arduino and SIM800L
  Serial.print(".");
  //ssGSM.begin(9600);

  fonaSerial->begin(4800);
/*
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  
  Serial.print(F("FONA is OK "));
  Serial.print(F("Type: "));
  Serial.println(fona.type());

  while (!(fona.getNetworkStatus() == 1 or fona.getNetworkStatus() == 5 )) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("Phone connected");
*/

 /*
  Serial.print(".");
  ssGSM.listen();
  Serial.print(".");
  ssGSM.println("AT"); //Once the handshake test is successful, it will back to OK
  Serial.print(".");
  Serial.print(ssGSM.readString());
  updateSerialGSM();
  ssGSM.println("ATE0"); //Echo aus, das stört nur.
  Serial.print(".");
  Serial.print(ssGSM.readString());
  updateSerialGSM();
  ssGSM.println("AT+CPIN=8352"); //PIN
  Serial.print(".");
  Serial.print(ssGSM.readString());
  updateSerialGSM();
  Serial.print("-");
  delay(1000);
  Serial.print(ssGSM.readString());
  ssGSM.println("AT+CMGF=1"); //SMS-Format setzen
  Serial.print(".");
  Serial.print(ssGSM.readString());
  ssGSM.println("AT+CMGD=1,4"); //löschen aller SMS auf der Karte nach dem einlesen
  Serial.print(".");
  Serial.print(ssGSM.readString());
  updateSerialGSM();

// Also das funtioniert schon mal.
  ssGSM.println("AT+CMGS=\"+49phonenumber\"");
  Serial.print(ssGSM.readString());
  updateSerialGSM();
  ssGSM.print("Big Master Test"); 
  ssGSM.print(" - "); 
  ssGSM.print("Part 2"); 
  ssGSM.print((char)26);
  Serial.print(ssGSM.readString());
*/





  
  //mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  //updateSerial();
  //mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  //updateSerial();
  Serial.print(".");
  status = IMU.begin();
 if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  Serial.print(".");
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  Serial.print(".");
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  Serial.print(".");
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  Serial.print(".");
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  // enabling wake on motion low power mode with a threshold of 400 mg and
  // an accelerometer data rate of 15.63 Hz. 
  //IMU.enableWakeOnMotion(400,MPU9250::LP_ACCEL_ODR_15_63HZ);
  // attaching the interrupt to microcontroller pin 1
  //pinMode(1,INPUT);
  //attachInterrupt(1,wakeUp,RISING);

  Serial.print(".");

  //fona.sendSMS("+49phonenumber", "System up and running");

  
  Serial.println("done.");

  
  //while(1);

  
}


void loop()
{
  String res = "init";
  float LocLat = 0;
  float LocLng = 0;
  float runtime;
  float GPStime = 30000;
  char smsBuffer[250];
  uint16_t smslen;


  digitalWrite(8, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(9, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(10, LOW);    // turn the LED off by making the voltage LOW

  
  switch (runmode) {
    case 0:
    {
      Serial.println("Runmode 0");
/*
      if (fona.readSMS(1, smsBuffer, 250, &smslen)) {
        Serial.print("Len: ");
        Serial.println(smslen);  
        res = "x123";
        /*
        for ( int i = 0; i < smslen; i++){
          Serial.print(smsBuffer[i]);
          Serial.print(res.concat(smsBuffer[i]));
        }
        */
        Serial.println();
        Serial.print("Res: 123");
        Serial.println(res);  
        if (res.indexOf("Scharf") > 0)
        {
          runmode = 1;
          Serial.println("Switch to Runmode 1");
        }
        //fona.deleteSMS(1);
     //  }
      delay(500);

      /*
      res = ssGSM.readString();
      if (res.indexOf("+CMTI:") > 0) //haben wir eine SMS bekommen?
      {
        Serial.println("incoming SMS");
        ssGSM.println("AT+CMGR=1");
        res = ssGSM.readString();
        Serial.print ("SMS:");
        Serial.print(res);
        Serial.println (":");
        ssGSM.println("AT+CMGD=1,4"); //löschen aller SMS auf der Karte nach dem einlesen
        resX = ssGSM.readString();
        if (res.indexOf("Scharf") > 0)
        {
          runmode = 1;
          Serial.println("Switch to Runmode 1");
        }
      }
      */
      break;  
    }
    case 1:
    {
      Serial.println("Runmode 1");
      digitalWrite(10, HIGH);   // turn the LED on (HIGH is the voltage level)
      res = ssGSM.readString();
      if (res.indexOf("+CMTI:") > 0) //haben wir eine SMS bekommen?
      {
        Serial.println("incoming SMS");
        ssGSM.println("AT+CMGR=1");
        res = ssGSM.readString();
        Serial.print ("SMS:");
        Serial.print(res);
        Serial.println (":");
        ssGSM.println("AT+CMGD=1,4"); //löschen aller SMS auf der Karte nach dem einlesen
        //resX = ssGSM.readString();
        if (res.indexOf("Unscharf") > 0)
        {
          runmode = 0;
          Serial.println("Switch to Runmode 0");
        }
      }
      if (CheckMoving() > 0)
      {
        runmode = 2;
        Serial.println("Switch to Runmode 2");
        // GSM aus und GPS an
        ssGSM.stopListening();
        LocLat = 0;
        LocLng = 0;
        ssGPS.listen();
        rm2timer = millis();
      }
      break;  
    }
    case 2:
    {
      Serial.print("Runmode 2");
      digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
      while (ssGPS.available())
        gps.encode(ssGPS.read());
      if (gps.location.isValid())
      {
        LocLat = gps.location.lat();
        LocLng = gps.location.lng();
        Serial.print(" - "); 
        Serial.print(LocLat,6); 
        Serial.print(" - "); 
        Serial.print(LocLng,6); 
      }
      Serial.println();
      runtime = millis() - rm2timer;
      Serial.print("Runtime: ");
      Serial.println(runtime,2);
      if (runtime > GPStime) //nach 10 Minuten wechseln wir in runmode 3, bis dahin sollten wir eine GPS-Position haben!
      {
        runmode = 3;
        Serial.println("Switch to Runmode 3");
        // GPS aus und GSM an
        ssGPS.stopListening();
        ssGSM.listen();
      }
      break;
    }
    case 3:
    {
      Serial.print("Runmode 3 ");
      digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);
      // senden der Position über SMS
      Serial.print("Sending SMS ");
      Serial.print(ssGSM.readString());
      updateSerialGSM();
      ssGSM.println("AT+CMGS=\"+49phonenumber\"");
      Serial.print(ssGSM.readString());
      Serial.print(ssGSM.readString());
      Serial.print(ssGSM.readString());
      updateSerialGSM();
      ssGSM.print(LocLat,3); 
      ssGSM.print(" - "); 
      ssGSM.print(LocLng,3); 
      ssGSM.print((char)26);
      Serial.print(ssGSM.readString());
      updateSerialGSM();
      Serial.println("Switch to Runmode 2");
      runmode = 2;
      // GSM aus und GPS an
      ssGSM.stopListening();
      ssGPS.listen();
      rm2timer = millis();
      break;
    }
    case 99:
    {
      Serial.println("Runmode 99");

      CheckMoving();

      res = ssGSM.readString();
      Serial.print ("SMS:");
      Serial.print(res);
      Serial.println (":");
      break;
    }
  }
    

  

  //updateGyro();

  /*    
  if (millis() - start > 1000)
  {
      start = millis();
      //printDateTime(gps.date, gps.time);   
      Serial.print(awake); 
      Serial.println();
      awake = 0;
  }

  */
  delay(200);  
}



int CheckMoving()
{
int Moving;
float trigger;
  
  Serial.print("GetGyroDeltas");
  Moving = 0;
  trigger = 0;
  // read the sensor
  IMU.readSensor();
  if (IMU.getAccelX_mss() > 0.1)
  {
    trigger = IMU.getAccelX_mss();
    Moving = 1;
  }  
  if (IMU.getAccelY_mss() > 0.4)
  {
    trigger = IMU.getAccelY_mss();
    Moving = 2;
  }  
  if (IMU.getAccelZ_mss() > 0.05)
  {
    trigger = IMU.getAccelZ_mss();
    Moving = 3;
  }  
  
  if (IMU.getMagX_uT() > 0.05)
  {
    trigger = IMU.getMagX_uT();
    Moving = 4;
  }  
  if (IMU.getMagY_uT() > 0.05)
  {
    trigger = IMU.getMagY_uT();
    Moving = 5;
  }  
  if (IMU.getMagZ_uT() > 31)
  {
    trigger = IMU.getMagZ_uT();
    Moving = 6;
  }

    
  if (IMU.getGyroX_rads() > 0.05)
  {
    trigger = IMU.getGyroX_rads();
    Moving = 7;
  }  
  if (IMU.getGyroY_rads() > 0.05)
  {
    trigger = IMU.getGyroY_rads();
    Moving = 8;
  }  
  if (IMU.getGyroZ_rads() > 0.05)
  {
    trigger = IMU.getGyroZ_rads();
    Moving = 9;
  }  
  Serial.print("Moving: ");
  Serial.print(Moving);
  Serial.print(" - ");
  Serial.print(trigger, 6);
  return Moving;
}



void GSMBufferClear()
{
  String trash;
  while(ssGSM.available()) 
  {
    trash = ssGSM.read();
  }
}


void updateGyro()
{
  
}

/*
void wakeUp() {
  awake = 1;
  Serial.println("Awake!");
}
*/
void updateSerialGSM()
{
  while (Serial.available()) 
  {
    ssGSM.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(ssGSM.available()) 
  {
    Serial.write(ssGSM.read());//Forward what Software Serial received to Serial Port
  }
}

/*
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
  printInt(d.age(), d.isValid(), 5);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
}
*/
