/*  pin allocation table- 
 *  (0,1)-GPS Rx/Tx
 *  (SDA,SCL)-IMU, magneto,slave arduinos
 *  3-throttle servo
 *  4-steering servo
 *  9,10,11,12,13- used for SPI by optical flow
 *  free pins- 2,5,6,7,8.
*/

/*NOTE: 
 * It would appear to you that a lot of the variables are global even though most would consider such a practice as unsafe. 
 * However on closer inspection, you will notice that the variables I have kept global are the ones that are shared between multiple functions.
 * This makes writing the functions etc easier for me, plus it reduces the function overhead as the variables are already in the working memory.
 * This also gives me a lot of freedom to use intermediate values from one function in another function(like accelerations, gyrations, etc) for filtering purposes.
 * It would be of some help if someone could tell me how i could make some variables read-only outside of a code block so that the system becomes a little more "secure"
 * without creating classes.
 */
 
/* recent updates(latest first)-
 *  motorWrite has it's own separate tab
 *  driver module has it's own separate tab
 * the function prototypes of separate tabs has been put into the main tab because the arduino IDE thinks i am writing in avr level and it wont let me define things in other tabs. 
 * usr module added 
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

/*
 * general architecture - collect data first ,then do all the math, then write all the signals
 * 
 */

#include "I2Cdev.h"       //communication libraries
#include <SPI.h>
#include "Wire.h"
#include "MPU9150.h"

#define PULL_HIGH B00011000
#define PULL_LOW B11100111
#define MOTOR_PIN B11110111 //3
#define STEERING_PIN B11101111 //4
#define dt 0.02


//--------------IMU STUFF-------------------------
MPU9150 accelgyro;
int16_t a[3],g[3];
uint16_t m[3];
uint8_t buf[6];
float A[3],G[3],M[3],lastA[3],lastG[3];
//-410.52  462.20  15889.75  -69.93  48.40 80.66
float offsetA[3]={-471,1100,16176},offsetG[3]={-32,23.37,39},offsetM[3]={44174,33516,36478};//34816||40574||36478|
float roll,pitch,yawRate;
float magbuf[2]={0,0};
float T[2];
float Ha,V=0;
//--------------IMU STUFF ENDS--------------------

//--------------GPS STUFF-------------------------
float latitude=0,longitude=0,Hdop=100000;
float iLong,iLat,destLat,destLong;
float destX,destY;
bool tick=false;
#define GPSBAUD 57600 
#define MAXVARIANCE 10//max hdop allowed during startup
//--------------GPS STUFF ENDS--------------------

//--------------OPTICAL FLOW STUFF----------------
SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3);    // 2 MHz, mode 3
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
bool connection=false;
//--------------OPTICLA FLOW STUFF ENDS-----------

//--------------ULTRASONIC STUFF------------------
volatile unsigned long timer[4];
volatile byte last_channel[3]={0,0,0};
volatile int input[3]={0,0,0};
// all this stuff is redundant for the time being until I add the obstacle avoidance unit (which uses a separate microcontroller 
//of it's own to handle the sensor management. 
float retard=0;    //braking
float sonarCorrection=0;  //deviation suggested by the sonar 
int message[2];
#define SONAR_MODULE_ADDRESS 1
#define SONAR_GAIN 3
#define RETARD_GAIN 0.2
#define MAXSONAR_TRIGGER 2
#define MAXSONAR_VCC 5
//--------------ULTRASONIC STUFF ENDS-------------

//--------------GENERAL VARIABLES-----------------
float ml,mh; //ml=slope of line connecting bot to destination, mh= slope of line along heading.
float d; // distance between bot and destination
int l=1;  //l- stopping distance from target. tick- variable used to ensure that we don't add gps and imu values twice (look in localization tab)
float globalX,globalY,X,Y;     //globalX and globalY are the final estimates of X and Y. theta is the angle rotated between consequent readings of gps,m is a variable for convenience 
float correction=0;  //deviation required by the bot
float now;  //variables for checking the time of the control loop
float estimateError,KG;  //imu's error in estimation and kalman gain
int point=1,i;
bool accelRequired=false;
//--------------GENERAL VARIABLES END-------------

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


void setup()
{
  Serial.begin(GPSBAUD); // initialize UART communication
  //-----------------OPTICAL FLOW Setup-----------------
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
  spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch-this is not my comment.
  //-----------------optical flow setup-----------------------
  
  //-------------- IMU setup begins-----------------
  Wire.begin();//initialize TWI communication
  TWBR=12;//prescaler set for 400KHz
  accelgyro.initialize();
  while(!accelgyro.testConnection())
  {  
    accelgyro.initialize();
    delay(50);
  }
  accelgyro.getMotion6( &a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
  for(int i = 0;i<3;i++)
  {
    for(int i=0;i<3;i++)
    {
      A[i] = a[i];
      A[i] -= offsetA[i];
      A[i] *= 0.0006103;
      lastA[i] = A[i];
  
      G[i] = g[i];
      G[i] -= offsetG[i];
      G[i] *= 0.007633;
      lastG[i] = G[i];
    }
    readMagDuringSetup();
    //the following formula has X and Y inverted but it works so don't touch it. 
    T[0]= pitch = 57.3*asin(A[1]*0.102); //0.102 = 1/9.8
    T[1]= roll  = -57.3*asin(A[0]*0.102);  //multiply by 57.3 to standardize roll and pitch into degrees 
    mh = tilt_Compensate(roll/57.3,pitch/57.3); //roll, pitch have to be in radians
    yawRate = G[2];
  }
  //------------IMU SETUP ENDS------------------------
  
  //----------------GENERAL SETUP BEGINS--------------
  i=3; // initialize i at 3.
  do
  {
    localizer();
    tick =false; //just something i have to do to make sure it doesn't take the last known GPS reading when it actually starts looping.
    iLat=latitude;
    iLong=longitude;
    if(Hdop<pow(10,i)) //while Hdop(which is not really Hdop, it is hAcc which is 3*2DAcc(from observation), sorry to the GPS lingo nuts) is greater than
    {                  //1000,100,10 keep checking the gps. an Hdop(hAcc) of 10(~3.3m 2DAcc) is basically the bare minimum I would want for decent localization.
      i--;
    }
    delay(200);     //wait 200ms because the gps updates at a rate of 5Hz so it would be kinda pointless to check in between.
  }
  while(Hdop>MAXVARIANCE); //while HDOP>10
  
  c[0].setcords(iLat,iLong);  //STORE LOCATION OF ORIGIN IN C[0]
  //for now since i m working in a local frame, i can use meters instead of GPS coordinates, This is a little more reliable too as the GPS probably can't
  //tell me the exact initial location in degrees anyway.
  c[0].setcordsO(0.0,0.0);   //the location of origin is stored in an object of class coordinates using a member function called setcords(float x,float y)   
  c[1].setcordsO(0.0,-3.0);   //location of destination:-can be anything, is stored in another object of class coordinates
  c[2].setcordsO(3.0,0.0);
  c[3].setcordsO(0.0,0.0); //setting the final location as initial location to check error.
  //setting the first destination coordinates.
  destX=c[1].retXO();// use (destLong-iLong)*111692.84 if working with degrees
  destY=c[1].retYO();//(destLat-iLat)*111692.84;
     
  X=globalX=0;  //initializing globalX and globalY
  Y=globalY=0;
  //--------------GENERAL SETUP ends-----------------------  
  //--------------MOTOR and STEERING initialization--------
  DDRD |= PULL_HIGH;
  PORTD |= PULL_HIGH;
  delayMicroseconds(1480);
  PORTD &=PULL_LOW;
  //---------------MOTOR and STEERING initialized----------
}



void loop()
{
  now = millis();
  Compute_Steering_Distance();//look into the "localization" tab.
  driver();
  while(millis()-now<=20); //fixing loop time.
}
