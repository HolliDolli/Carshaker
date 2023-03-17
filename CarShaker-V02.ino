
#include <MemoryFree.h>
#include <SoftwareSerial.h>
#include <Sim800l.h>
#include <TinyGPS++.h>
#include <MPU9250.h>



// runmodes:
// 0 unarmed, waiting for SMS to arm
// 1 armed, waiting for gyro to alert or SMS to disarm
// 2 alert, reading GPS, after 9 Min going to runmode 3
// 3 send, sending last good GPS Position and return to runmode 2
// 99 debugging , only execute statements in block

int runmode = 1;
//char targetphone[15] = "+49phonenumber";
char targetphone[15] = "+49phonenumber2";

float LocLat = 0;
float LocLng = 0;
bool LocValid = false;
float MovMin[9] = { 99, 99, 99, 99, 99, 99, 99, 99, 99};
float MovMax[9] = {-99,-99,-99,-99,-99,-99,-99,-99,-99};
bool error;

MPU9250 IMU(Wire,0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Sim800l Sim800l;  //to declare the library


void setup() {
  // put your setup code here, to run once:
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(115200);
  Sim800l.begin();

  //Serial.print(F("Initializing"));
  //dbOut(F("CarShaker"),F("Init"),true);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(5, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(5, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(7, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(9, LOW);    // turn the LED off by making the voltage LOW
  if (LocValid) {
    //digitalWrite(11, LOW);    // turn the LED off by making the voltage LOW
  }
  //dbOut(F("loop FreeMem"),String(freeMemory()),true);
  //Serial.println(F("loop"));
  switch (runmode) {
    case 0:
    {
      runmode0a1();
      break;
    }
    case 1:
    {
      runmode0a1();
      break;
    }
    case 2:
    {
      runmode2();
      break;
    }
    case 3:
    {
      runmode3();
      break;
    }
  }
}

void dbOut(String Source, String Msg, bool LF){
  if (Source.length() > 0) {
    Serial.print(Source);
    Serial.print(F(" - "));
  }
  Serial.print(Msg);
  if (LF) {
    Serial.println();
  }
}

void runmode3() //runmode 3: transfer location via SMS.
{

char cMes[80];
String Message;
  
  
  //dbOut(F("rm3 FreeMem"),String(freeMemory()),true);
  digitalWrite(5, HIGH);   // turn the LED on (HIGH is the voltage level)
  Sim800l.begin();

  Message = String("CarShake Position : " + String(LocLat) + ":" + String(LocLng));
  Message.toCharArray(cMes, 80);
  error = false;
  for (int I = 1; I < 6 && !error; I++) {
    //dbOut("rm3", Message, true);
    error = Sim800l.sendSms(targetphone, cMes);
    delay(3000); //just to give some time before next try
  }
  runmode = 2;
}



void runmode2() //runmode 2 only getting GPS-Position.
{
SoftwareSerial ssGPS(3, 2); //GPS Tx & Rx is connected to Arduino #7 & #6
TinyGPSPlus gps; // The TinyGPS++ object

float runner = millis();
float GPStime = 300000;

//  dbOut(F("Runmode2"),F("Start"),true);
//  dbOut(F("rm2 FreeMem"),String(freeMemory()),true);
  digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)
  //dbOut(F("rm2 FreeMem"),String(freeMemory()),true);
  //error = Sim800l.sendSms(targetphone, "CarShake Alert!");
  Sim800l.begin();
  Sim800l.callNumber(targetphone);
  for (int i = 1; i < 21; i++){  //Achtung: gerade Anzahl von Wechseln, sonst licht falsch!
    delay(1000);
    digitalWrite(7, !digitalRead(7));   // Toogle LED
  }
  //error = Sim800l.hangoffCall();
  ssGPS.begin(9600);
  while (millis() < runner + GPStime) {
    while (ssGPS.available())
      //dbOut("", "#",false);
      //Serial.print(".");
      gps.encode(ssGPS.read());
    if (gps.location.isValid()) {
      //dbOut(F("Runmode2"),F("ValidLocation!"),true);
      LocLat = gps.location.lat();
      LocLng = gps.location.lng();
      LocValid = true;
    }
  }
  if (LocValid) {   //only if we got valid location switch to mode 3 (transfer) else stay in mode 2
    runmode = 3;
  }
  //runmode = 3; //only for debugging.

}


void runmode0a1() //runmode 0 and 1 are nearly equal.
{
String text;
bool running = true;
char phone[25];
int status = -1;
int Moving = 0;
float runner = millis();


  //dbOut(F("Runmode0/1"),F("Start"),true);
  //Serial.println(F("Runmode 0/1 a"));
  //dbOut(F("rm1 FreeMem"),String(freeMemory()),true);
  //dbOut(F("Runmode0/1"),F("Init sim800 ."),false);
  Sim800l.begin();
  //dbOut("", F("done"), true);
  delay(200);
  //Serial.println(F("Runmode 0/1 b"));

  if (runmode == 1) {
    digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
    status = IMU.begin();
    if (status >= 0) {
      IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);   // setting the accelerometer full scale range to +/-8G 
      IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);  // setting the gyroscope full scale range to +/-500 deg/s
      IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);  // setting DLPF bandwidth to 20 Hz
      IMU.setSrd(19);  // setting SRD to 19 for a 50 Hz update rate
      InitMoving();
    }
    else {
      error = Sim800l.sendSms(phone, "CarShake Error MPU9250, CarShake OFF!");
      running = false;
      runmode = 0;
    }
  }
  //Serial.println(F("Runmode 0/1 c"));
  runner = millis();
  while (running) {
    dbOut(F("Runmode0/1"), F("loop"),true);
//    dbOut(F("rml FreeMem"),String(freeMemory()),true);
    if (runmode == 1) {
      Moving = CheckMoving();
      if (Moving != 99) {
//        dbOut(F("Runmode1"),F("Switching to Runmode 2"),true);
        running = false;
        runmode = 2;
      }
    }
    if (millis() > runner + 500) { //Nur alle 500ms schauen wir nach der SMS.
      digitalWrite(9, !digitalRead(9));   // Toogle LED
      Serial.println(F("CheckSMS"));
      text=Sim800l.readSms(1);
      dbOut(F("SMS:"), text, false);
//      dbOut("", F("- SMSEnd"),true);
      if (runmode == 0 && text.indexOf(F("CSon")) > 0) {
//        dbOut(F("Runmode0"),F("Switching to Runmode 1"),true);
        runmode = 1;
        running = false;
        error = Sim800l.sendSms(phone, "CarShake ON!");
      }
//      if (runmode == 0) {
//        delay(30000); //da das mit der AktivierungsSMS nicht klappt geben wir 30 Sekunden bevor wir scharf schalten.
//        runmode = 1;
//        running = false;
//        error = Sim800l.sendSms(phone, "CarShake ON!");
//      }

      if (runmode == 1 && text.indexOf(F("CSoff")) > 0) {
//        dbOut(F("Runmode1"),F("Switching to Runmode 0"),true);
        runmode = 0;
        running = false;
        error = Sim800l.sendSms(phone, "CarShake OFF!");
      }
      Sim800l.delAllSms();
      runner = millis();  
      //Serial.println(F("CSend"));
    }
  }
}

void InitMoving()
{
float trigger;
int I;
  for (int ix = 1; ix<=10; ix++){
    IMU.readSensor();
    I = 0;
    trigger = IMU.getAccelX_mss();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 1;
    trigger = IMU.getAccelY_mss();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 2;
    trigger = IMU.getAccelZ_mss();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 3;
    trigger = IMU.getMagX_uT();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 4;
    trigger = IMU.getMagY_uT();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 5;
    trigger = IMU.getMagZ_uT();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 6;
    trigger = IMU.getGyroX_rads();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 7;
    trigger = IMU.getGyroY_rads();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    I = 8;
    trigger = IMU.getGyroZ_rads();
    if (trigger < MovMin[I]) MovMin[I] = trigger;
    if (trigger > MovMax[I]) MovMax[I] = trigger;
    delay(150);
    digitalWrite(9, !digitalRead(9));   // Toogle LED
  }
    /*
    for (int i = 0; i < 9; i++){
      Serial.println(MovMin[i]);
      Serial.println(MovMax[i]);
    }*/
    MovMin[0] -= 0.2;
    MovMax[0] += 0.2;
    MovMin[1] -= 0.2;
    MovMax[1] += 0.2;
    MovMin[2] -= 0.2;
    MovMax[2] += 0.2;
    MovMin[3] -= 3;
    MovMax[3] += 3;
    MovMin[4] -= 3;
    MovMax[4] += 3;
    MovMin[5] -= 3;
    MovMax[5] += 3;
    MovMin[6] -= 0.2;
    MovMax[6] += 0.2;
    MovMin[7] -= 0.2;
    MovMax[7] += 0.2;
    MovMin[8] -= 0.2;
    MovMax[8] += 0.2;
    /*
    for (int i = 0; i < 9; i++){
      Serial.println(MovMin[i]);
      Serial.println(MovMax[i]);
    }*/
}

int CheckMoving()
{
int Moving;
float trigger;
int I;
float Wert;

  //dbOut(F("mv1 FreeMem"),String(freeMemory()),true);
  //dbOut(F("CheckMoving"), F("GetGyroDeltas"), true);
  //Serial.println(F("Checkmoving"));
  Moving = 99;
  trigger = 0;
  // read the sensor
  IMU.readSensor();
  Wert = IMU.getAccelX_mss();
  I = 0;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getAccelY_mss();
  I = 1;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getAccelZ_mss();
  I = 2;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
/*  
 *  Die Magnetfeldsensoren sind nicht sauber (bzw. ich habe es noch nciht kapiert. :-) 
 Wert = IMU.getMagX_uT();
  I = 3;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getMagY_uT();
  I = 4;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getMagZ_uT();
  I = 5;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  */
  Wert = IMU.getGyroX_rads();
  I = 6;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getGyroY_rads();
  I = 7;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
  Wert = IMU.getGyroZ_rads();
  I = 8;
  if (Wert > MovMax[I] || Wert < MovMin[I]) {
    Moving = I;
    trigger = Wert;
  }
 
//  dbOut(F("Moving: "), String(Moving), true);
//  dbOut(F("Trigger: "), String(trigger), true);
//  dbOut(F("mv2 FreeMem"),String(freeMemory()),true);
  //Serial.println(Moving);
  if (Moving != 99 ){
    //Serial.println(MovMin[Moving]);
    //Serial.println(MovMax[Moving]);
    //Serial.println(trigger);
  }
  //Serial.println(F("CMend"));
  return Moving;
}
