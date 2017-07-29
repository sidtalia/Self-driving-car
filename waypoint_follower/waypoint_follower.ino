/*  pin allocation table- 
 *  (0,1)-GPS Rx/Tx
 *  (SDA,SCL)-IMU, magneto,slave arduinos
 *  3-throttle servo
 *  4-steering servo
 *  9,10,11,12,13- used for SPI by optical flow
 *  free pins- 2,5,6,7,8.
*/

/* recent updates(latest first)-
 * code redundancy reduced 
 * usr module added 
 * miscellaneous changes- mh=90 initialization removed 
 *                       - global coordinates updated each cycle  
 *                       - math functions are in a separate tab 
 *                       - k(accel trust) was for some reason not being calculated, so added a statement for that
 * filter updated to handle gps and opflow instead of gps and imu.
 * i2c communication to slave arduinos added.function takes address of slave and the message array into which the message is stored. currently only for 2 pieces of information. Not in use right now
 * added optical flow code .  it's function is called in the localization tab before calling "callimu()". results from OpFlow are used in accelgyro tab 
 * magnetometer incorporated. function call is in accelgyro tab
 * accelgyro offsets are now fixed. cant use #define stuff because it would not allow the call imu code to work 
 * accelgyro gps kalman tested.
 * accelgyro incorporated. function call is in the localization tab
 * gps update rate increased to 5Hz, now using ublox's own protocol which has much smaller packet size, baud rate kept at 57600, can be increased to 230400.
 * gps used for basic localization. 
*/

#include <Servo.h>  //servo driving library- Caution, it uses interrupts to run servos.

#include "I2Cdev.h"       //communication libraries
#include <SPI.h>
#include "Wire.h"

#include "MPU6050.h"              //sensor libraries 
#include <HMC5883L.h>

#define LED 7
//-----------SERVOS----------------
Servo steer,throttle;  //servo objects
#define STEERINGPIN 4
#define THROTTLEPIN 3
#define STEERING_NULL 1440
#define THROTTLE_NULL 1460
//----------------------------------

//-----ACCEL-GYRO STUFF BEGINS---------------------
MPU6050 accelgyro;    //mpu6050 object
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t a[3]; //accelerations from mpu6050
int16_t g[3];  //gyration rates from mpu6050

float A[3],G[3],lastA[3]={0,0,0},lastG[3]={0,0,0},offsetA[3]={151,(-70),15385},offsetG[3]={(-424),119,62.34};
float T[2]={0,0},V=0,X=0,Y=0,Ha;  //x=0,y=1,z=2, T=tilt,V=velocity ,X=position on X axis,Y=position on Y axis,Ha=horizontall acceleration along Y direction
int i;
bool connection =0 ;

//-----------ACCEL-GYRO STUFF ENDS----------------

//----------------MAGNETOMETER STUFF BEGINS------------------
HMC5883L compass;    //initializing compass object
#define MOffX -23
#define MOffY -99
Vector raw ;
//----------------MAGNETOMETER STUFF ENDS--------------------

//------GPS STUFF---------------------------------
float latitude=0,longitude=0,Hdop=100000;
float iLong,iLat,destLat,destLong;
float destX,destY; 
#define GPSBAUD 57600 
#define MAXVARIANCE 10//max hdop allowed during startup
//------GPS STUFF ENDS----------------------------

//----------------OPTICAL FLOW STUFF----------------
SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3);    // 2 MHz, mode 3

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

#define RESET_PIN 6
#define SS_PIN 10 // Pin 10

int8_t dx,dy;            
uint8_t surfaceQuality;   

//----------------OPTICAL FLOW STUFF ENDS-----------




//----------------ULTRASONIC MODULE STUFF-------------
volatile unsigned long timer[4];
volatile byte last_channel[3]={0,0,0};
volatile int input[3]={0,0,0};

float retard=0;    //braking
float sonarCorrection=0;  //deviation suggested by the sonar 
int message[2];
#define SONAR_MODULE_ADDRESS 1
#define SONAR_GAIN 3
#define RETARD_GAIN 0.2
#define MAXSONAR_TRIGGER 2
#define MAXSONAR_VCC 5
//----------------ULTRASONIC MODULE STUFF ENDS--------



//-----general program variables------------------
float ml,mh; //ml=slope of line connecting bot to destination, mh= slope of line along heading.
float d; // distance between bot and destination
int l=1;  //l- stopping distance from target. tick- variable used to ensure that we don't add gps and imu values twice (look in localization tab)
float globalX,globalY;     //globalX and globalY are the final estimates of X and Y. theta is the angle rotated between consequent readings of gps,m is a variable for convenience 
float correction=0;  //deviation required by the bot
float now,dt=0.0;  //variables for checking the time of the control loop
float estimateError,KG;  //imu's error in estimation and kalman gain
int point=0;
bool accelRequired=false,tick=false;
//-------------------------------


class coordinates //for storing multiple coordinates as we further progress to multiple waypoint trajectories instead of just 1 waypoint
{                               //this will become the main point of focus once the basic tasks are complete
  float longitude,latitude,X,Y;
  public:
  void setcords(float y,float x)
  {
    latitude=y;
    longitude=x;
  }
  float rety()
  {
    return latitude;
  }
  float retx()
  {
    return longitude;
  }
  float setcordsO(float y,float x)
  {
    X=x;
    Y=y;
  }
  float retYO()
  {
    return Y;
  }
  float retXO()
  {
    return X;
  }
  
};

coordinates c[4];


//----------------SETUP BEGINS-------------------
void setup()
{
  Serial.begin(GPSBAUD);
  delay(2000);

  //=======ULTRASONIC SETUP BEGINS===================
  pinMode(MAXSONAR_VCC,OUTPUT);
  rebootSensors();   //credits to Davis Devasia 
  
  PCICR  |= (1 << PCIE1);                     // Configure pin change interrupts for PCINT1
  PCMSK1 |= (1 << PCINT8);                    // Pin change interrupt for A0
  PCMSK1 |= (1 << PCINT9);                    // Pin change interrupt for A1
  PCMSK1 |= (1 << PCINT10);                   // Pin change interrupt for A2
  PCMSK1 |= (1 << PCINT11);
  
  //=======ULTRASONIC SETUP ENDS=====================
  pinMode(LED,OUTPUT); //debug LED
  //========OPTICAL FLOW SETUP BEGINS================ 
  SPI.begin();
  // Set SS and reset pin as output
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset();

  while(!connection)
  { 
    uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
    (id == ADNS3080_PRODUCT_ID_VALUE)? connection=1 : connection=0;  // represents connection break  
  }
  uint8_t config = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch

  for(i=0;i<3;i++)
  {
    digitalWrite(LED,HIGH);
    delay(150);
    digitalWrite(LED,LOW);
    delay(150);
  }
 
  delay(1000);
  //---------OPTICAL FLOW SETUP ENDS-----------------  

    //===========ACCELGYRO SETUP BEGINS=============== 
  Wire.begin();
  connection=0;
  while(!connection)
  {
    accelgyro.initialize();
    accelgyro.testConnection() ? connection=1 : connection=0 ; 
  }
  
  for(i=0;i<3;i++)
  {
    digitalWrite(LED,HIGH);
    delay(150);
    digitalWrite(LED,LOW);
    delay(150);
  }

  delay(1000);
  //------------ACCEL-GYRO SETUP ENDS-------------
  
  //============COMPASS CALIB BEGINS==============
  while (!compass.begin())
  {
    delay(5);
  }
  compass.setRange(HMC5883L_RANGE_8_1GA);  //range 8.1 gauss
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  //mode- continuous 
  compass.setDataRate(HMC5883L_DATARATE_75HZ);  //data coming in at 75 Hz
  compass.setSamples(HMC5883L_SAMPLES_1);  //samples averaged for normalized values
  compass.setOffset(MOffX,MOffY);  //providing offsets 
  //-------------COMPASS CALIB ENDS---------------
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  delay(1000);
  i=3;
  //==============GPS SETUP BEGINS=================
  do
  {
    localizer();
    iLat=latitude;
    iLong=longitude;
    if(Hdop<pow(10,i))
    {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(100);
      i--;
    }
  }
  while(Hdop>MAXVARIANCE);
  c[0].setcords(iLat,iLong);  //STORE LOCATION OF ORIGIN IN C[0]
  c[0].setcordsO(0.0,0.0);   //the location of origin is stored in an object of class coordinates using a member function called setcords(float x,float y)   
  c[1].setcordsO(0.0,-3.0);   //location of destination:-can be anything, is stored in another object of class coordinates
  c[2].setcordsO(3.0,0.0);
  c[3].setcordsO(0.0,0.0);

  destX=c[1].retXO();//(destLong-iLong)*111692.84;
  destY=c[1].retYO();//(destLat-iLat)*111692.84;
     
  X=globalX=0;  //initializing globalX and globalY
  Y=globalY=0;
  for(i=0;i<100;i++)
  {
      raw=compass.readRaw();
      if(atan2(raw.YAxis,-raw.XAxis)<0)
      {
        mh+= ( 360 + 57.3*atan2(raw.YAxis,-raw.XAxis) );
      }
      else
      {
        mh+= 57.3*atan2(raw.YAxis,-raw.XAxis);
      }
  }
  mh/=100;
  
  
  //--------INITIAL LOCATION AND HEADING FOUND------------------------

  //====ATTACH STEERING AND THROTTLE SERVOS=========
  steer.attach(STEERINGPIN,1000,2000);
  throttle.attach(THROTTLEPIN,1000,2000);
  steer.writeMicroseconds(STEERING_NULL);
  throttle.writeMicroseconds(THROTTLE_NULL);     
  //-----------THROTTLE STEERING SETUP DONE-------
  
  //--------GPS SETUP ENDS------------------------
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
}

//--------SETUP ENDS------------------------------------
bool state=true;
inline void driver()   //p controller definition 
{
  if(d<l)
  {
    point++;
    if(point>=4)
    {
      throttle.writeMicroseconds(1000);  //if within 1 meter of target, stahp
      while(1);
    }
    else
    {
      destY=c[point].retYO();
      destX=c[point].retXO();
    }
  }
  else
  {
    throttle.writeMicroseconds(1500-retard);  // low speed
  }
  if(mod(sonarCorrection)>80)
  {
    steer.writeMicroseconds(STEERING_NULL-2*correction+(sonarCorrection)-G[2]);  //this is a PD algo for steering.    
  }
  else if(mod(sonarCorrection)<=80)
  {
    steer.writeMicroseconds(STEERING_NULL-3*correction+0*(sonarCorrection/3)-G[2]); //increase localization's correction gain in case sonar correction is smaller than 5 degrees
  
  }
}



void loop()
{
 dt=(millis()-now)/1000.0f;
 now=millis();  //start clock
 gpsSuggest();
 
// callSonar();
 driver();
}

ISR(PCINT1_vect)
{
  timer[0]=micros();
  //channel 1 ----
  
  if(last_channel[0]==0&& PINC & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[0]=1;
    timer[1]=timer[0];           
  }
  else if(last_channel[0]==1 && !(PINC & B00000001))
  {
    last_channel[0]=0;
    input[0]=timer[0]-timer[1];
  }

  //channel 2---                  
  if(last_channel[1]==0 && PINC & B00000010) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[1]=1;
    timer[2]=timer[0];          
  }
  else if(last_channel[1]==1 && !(PINC & B00000010))
  {
    last_channel[1]=0;
    input[1]=timer[0]-timer[2];
  }
  
  //channel 3-- 
  if(last_channel[2]==0&& PINC & B00000100) //makes sure that the first pin was initially low and is now high
  {                                         //PINC & B00000001 is equivalent to digitalRead but faster
    last_channel[2]=1;
    timer[3]=timer[0];          
  }
  else if(last_channel[2]==1 && !(PINC & B00000100))
  {
    last_channel[2]=0;
    input[2]=timer[0]-timer[3];
  }
}

