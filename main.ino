#include <checksum.h>
#include <mavlink.h>
#include <mavlink_helpers.h>
#include <mavlink_types.h>
#include <protocol.h>
unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000;
const int num_hbs = 60;
int num_hbs_pasados = num_hbs;
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_HIL_STATE 90

/*
  ======================================
  -Autonomous Surface Vehicle Prototype-
  By Alif Ihza Ahmada for Undergraduate Final Project
  Written in Arduino IDE using STM32duino Core
  MCU used in this project is STM32F103C8
  The MCU is installed with HID bootloader 2.2
  ======================================
*/

#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <LIS3MDL.h>
//#include <Kalman.h>

#define wpCount 7

HardwareSerial serialGps(PA3, PA2);
HardwareSerial GCS(PB11, PB10);

TinyGPSPlus gps;
Servo bldc;
Servo servo;
MPU6050 mpu;
LIS3MDL mag;
//Kalman kalman;

float gxr, gyr, gzr,
      pitch, roll,
      aPitch, aRoll,
      gPitch, gRoll;

float gyro[2], compass[2], accel[2], tuning[4],
      jarak, bearing, heading, v, error;

float mavP, mavR;

double Lat, Lon, latB, lonB, wpLat, wpLon, alt, x, y, s, a1, a2,
       elapsedTime,
       pi = 3.14159;

double point[10], WP[20], er[2];;

int pwm1=1000, pwm2, rudder, sat, pwmPid, Start, Emergency, rSpeed;
int ch1 = PB15, ch2 = PB14, ch3 = PB13, ch4 = PB12;
//char c;
String dataIn;
int Index[32], targetWp = 0, direction = 1;

const byte dataByte = 255;
char dataRec[dataByte];
char tempChar[dataByte];
char dataGCS[dataByte] = {0};
bool newData = false;
int parsing;


typedef struct Rumus {
  double dis;
  double azimuth;
} rumus;

String wp1[8] = {"-7.0514528", "110.4285729", "-7.0519000", "110.4292864", "-7.0514395", "110.4295975", "-7.0509710", "110.4288733"};
double wp2[8] = { -7.052584, 110.445956, -7.052605, 110.44581, -7.0496826, 110.4250847, -7.0496427, 110.4249814};
bool cek = true;

unsigned long time, prevTime;
float detik;

//remote
unsigned long channel[4];
uint32_t rcStart[4];
uint16_t rcValue[4];
volatile uint16_t rcShared[4];

void setup() {
  delay(500);
  Serial.begin(115200);
  serialGps.begin(9600);
  GCS.begin(57600);
  Wire.begin();
  //initMpu();
  initKompas();
  pinMode(PB1, OUTPUT);
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  bldc.attach(PA6);
  servo.attach(PA7);
  servo.writeMicroseconds(1500);
  bldc.writeMicroseconds(1000);

  attachInterrupt(ch1, rem1, CHANGE);
  attachInterrupt(ch2, rem2, CHANGE);
  attachInterrupt(ch3, rem3, CHANGE);
  delay(3000);
}

void loop() {
  detik = time;
  time = millis();
  elapsedTime = (time - detik) / 1000;
  rcReadVal();
  //  bacaCompass(0x0D);

  //  mavlink();
  //  Mav_Request_Data();
  //  comm_receive();
  recData();

  //bacaMpu();
  bacaGps();
  bacaKompas();

  
  algoritma();
  if (rcValue[2] > 1500) {
    pwm1 = map(rcValue[0], 1490, 2000, 1000, 1400);
    pwm2 = map(rcValue[1], 1000, 1990, 2150, 800);
  }
  if (rcValue[2] < 1500) {
    if(Start) {
      pwm1 = rSpeed;
      pwm2 = rudder;
    }
    else {
      pwm1 = 1000;
    }
  }
  //vincenty();
  if (time - prevTime >= 200) {
    prevTime = time;
    tampilan();
    //trial();
    kirim();
    latB = Lat;
    lonB = Lon;
  }

  if(pwm1>1500) pwm1=1500;
  bldc.writeMicroseconds(pwm1);
  servo.writeMicroseconds(pwm2);
}
