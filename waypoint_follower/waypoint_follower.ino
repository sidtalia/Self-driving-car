/*  pin allocation table- 
 *  (0,1)-GPS Rx/Tx
 *  (SDA,SCL)-IMU, magneto,slave arduinos
 *  3-throttle servo
 *  4-steering servo
 *  9,10,11,12,13- used for SPI by optical flow
 *  free pins- 2,5,6,7,8.
*/

/* recent updates(latest first)-
 *  usr module added 
 * miscellaneous changes- mh=90 initialization removed 
 *                       - global coordinates updated each cycle  
 *                       - math functions are in a separate tab 
 *                       - k(accel trust) was for some reason not being calculated, so added a statement for that
 * Kalman filter updated to handle gps and opflow instead of gps and imu.
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

#include "MPU6050.h"              //sensor libraries 
#include <HMC5883L.h>



//-----------SERVOS----------------
Servo steer,throttle;  //servo objects
#define STEERINGPIN 4
#define THROTTLEPIN 3
#define STEERING_NULL 1480
#define THROTTLE_NULL 1480
//----------------------------------


//-----ACCEL-GYRO STUFF BEGINS---------------------
 
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;  //accelerations from mpu6050
int16_t gx, gy, gz;  //gyration rates from mpu6050

//#define OUTPUT_READABLE_ACCELGYRO

float A[3],G[3],lastA[3],lastG[3],offsetA[3],offsetG[3];
float T[3],V[3],X=0,Y=0,Ha;  //x=0,y=1,z=2, T=tilt,V=velocity ,X=position on X axis,Y=position on Y axis,Ha=horizontall acceleration along Y direction
float k;  //trust factor
int i,connection=1;    //test connection and run samples

//-----------ACCEL-GYRO STUFF ENDS----------------


//----------------MAGNETOMETER STUFF BEGINS------------------
  HMC5883L compass;    //initializing compass object
  int MOffX=-48,MOffY=-189;
  Vector raw ;
//----------------MAGNETOMETER STUFF ENDS--------------------



//------GPS STUFF---------------------------------
float latitude,longitude;
int check=100;  //no. of gps cycles we will wait before we begin the actual program
float iLong,iLat,lastLong,lastLat,destLat,destLong;
float destX,destY; 
int counter=0;
float t=0;        //time between gps readings.
#define GPSBAUD 57600 
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
uint8_t accelRequired=0;

//----------------OPTICAL FLOW STUFF ENDS-----------




//----------------ULTRASONIC MODULE STUFF-------------
float retard=0;    //braking
float sonarCorrection=0;  //deviation suggested by the sonar 
int message[2];
#define SONAR_MODULE_ADDRESS 1
#define SONAR_GAIN 3
#define RETARD_GAIN 0.2
//----------------ULTRASONIC MODULE STUFF ENDS--------



//-----general program variables------------------
float ml,mh; //ml=slope of line connecting bot to destination, mh= slope of line along heading.
float d; // distance between bot and destination
int l=1,tick=0;  //l- stopping distance from target. tick- variable used to ensure that we don't add gps and imu values twice (look in localization tab)
float globalX,globalY,theta,m;      //globalX and globalY are the final estimates of X and Y. theta is the angle rotated between consequent readings of gps,m is a variable for convenience 
float correction=0;  //deviation required by the bot
float now,dt=0.0;  //variables for checking the time of the control loop
float estimateError,KG;  //imu's error in estimation and kalman gain
int point=1;
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

  //========OPTICAL FLOW SETUP BEGINS================ 
  SPI.begin();
  // Set SS and reset pin as output
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset();

  uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
  
  if (id == ADNS3080_PRODUCT_ID_VALUE)
  {
    connection=1;
  }
  else 
  {
    connection=0;  // represents connection break
  }
  
  uint8_t config = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch
  
  //---------OPTICAL FLOW SETUP ENDS-----------------  
  
  //====ATTACH STEERING AND THROTTLE SERVOS=========
  steer.attach(STEERINGPIN,1000,2000);
  throttle.attach(THROTTLEPIN,1000,2000);
  steer.writeMicroseconds(STEERING_NULL);
  throttle.writeMicroseconds(THROTTLE_NULL);     
  //-----------THROTTLE STEERING SETUP DONE-------

    //===========ACCELGYRO SETUP BEGINS===============
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.setSleepEnabled(false);
    accelgyro.initialize();
   // accelgyro.testConnection() ? connection=1 : connection=0 ; 
    //calculating offsets
    for(i=0;i<3;i++)
    {
      lastA[i]=0;  //kyunki karna padta hai
      lastG[i]=0; 
      T[i]=0;      //sabkuch initially 0 tha
      V[i]=0;
    }
    offsetA[0]=151.63;        //these offsets were calculated beforehand
    offsetA[1]=(-771.23); 
    offsetA[2]=15385.05;
    offsetG[0]=(-404.04);
    offsetG[1]=139.25;
    offsetG[2]=62.34;
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
  //-------------COMPASS CALIB ENDS----------------


  //==============GPS SETUP BEGINS=================
  while(longitude==0)     //waiting for the gps to start returning valid longitude and latitude values instead of 0s
  {
    while(!processGPS())
    {
      delay(200);
    }
    updategps();
  }
  //--------GPS SETUP ENDS------------------------
  
}

//--------SETUP ENDS------------------------------------

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
    steer.writeMicroseconds(STEERING_NULL-2*correction+(sonarCorrection));  //this is a P algo for steering.    
  }
  else if(mod(sonarCorrection)<=80)
  {
    steer.writeMicroseconds(STEERING_NULL-4*correction+(sonarCorrection/3)); //increase localization's correction gain in case sonar correction is smaller than 5 degrees
  }
  delay(4); //delay so that the servo writing gets enough time
}


void loop()
{
 dt=(millis()-now)/1000.0f;
 now=millis();  //start clock
 while(!connection)  //if connection failed during startup.
 {
      steer.writeMicroseconds(2000);
      delay(1000);
      steer.writeMicroseconds(1000);
      delay(1000);
      throttle.writeMicroseconds(1000);
 }
 gpsSuggest();
 driver();  // a P controller for throttle and steering.
}


