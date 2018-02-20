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
 * This makes writing the functions easier for me, plus it reduces the function overhead as the variables are already in the working memory.
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
 * psuedo Kalman filter updated to handle gps and opflow instead of gps and imu.
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
#include<Servo.h>

Servo motor,steer;
#define dt 0.0025 //cycle time


//--------------IMU STUFF-------------------------
MPU9150 accelgyro;
int16_t a[3],g[3];
uint16_t m[3];
uint8_t buf[6];
float A[3],G[3],M[3],lastA[3],lastG[3];
//413.97||764.10||16140.42||-134.85||94.52||162.61||

float offsetA[3]={400,764,16176},offsetG[3]={-32,23,40},offsetM[3]={44174,33516,36478};//offsets for IMU
float roll,pitch,yawRate;
float magbuf[2]={0,0};//buffer for magnetic strength values
float T[2]; //tilt(roll and pitch estimates using gyro
float Ha,V=0; //horizontal acceleration, velocity estimate
//--------------IMU STUFF ENDS--------------------

//--------------GPS STUFF-------------------------
float latitude=0,longitude=0,Hdop=100000;
float iLong,iLat,destLat,destLong;
float destX,destY;
bool tick=false;
#define GPSBAUD 115200
#define MAXVARIANCE 20//max hdop in meters allowed during startup
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
float setPoint=1.5,error =0,Serror =0;
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
  //--------------MOTOR and STEERING initialization-------
  delay(2000);

  //---------------MOTOR and STEERING initialized----------
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
  updateOpticalFlow();
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
  for(int i=0;i<3;i++)
  {
    A[i] = a[i];
    A[i] -= offsetA[i];
    A[i] *= 0.0006103;
    lastA[i] = A[i];

    G[i] = g[i];
    G[i] -= offsetG[i];
    G[i] *= 0.030516;
    lastG[i] = G[i];
  }
  
  readMagDuringSetup();
  pitch = 57.3*asin(A[1]*0.102); //0.102 = 1/9.8
  roll  = -57.3*asin(A[0]*0.102);  //multiply by 57.3 to standardize roll and pitch into degrees 
  mh = tilt_Compensate(roll/57.3,pitch/57.3); //roll, pitch have to be in radians
  yawRate = G[2];
  //------------IMU SETUP ENDS------------------------
  motor.attach(3);
  steer.attach(4);
  for(int i=0;i<250;i++)
  {
    motor.writeMicroseconds(1620);
    steer.writeMicroseconds(1480);
    delay(20);
  }
  //----------------GENERAL SETUP BEGINS--------------
  i=3; // initialize i at 3.
  do
  {
    localizer();
    tick =false; //just something i have to do to make sure it doesn't take the last known GPS reading when it actually starts looping.
    iLat=latitude;
    iLong=longitude;
    if(Hdop<pow(10,i)) //while Hdop(which is not really Hdop, it is hAcc, sorry to the GPS lingo nuts) is greater than
    {                  //1000,100,10 keep checking the gps. an Hdop(hAcc) of 10 is basically the bare minimum I would want for decent localization.
      i--;
    }
  }
  while(Hdop>MAXVARIANCE); //while HDOP>10
  
  c[0].setcords(iLat,iLong);  //STORE LOCATION OF ORIGIN IN C[0]
  //for now since i m working in a local frame, i can use meters instead of GPS coordinates, This is a little more reliable too as the GPS probably can't
  //tell me the exact initial location in degrees anyway.
  c[0].setcordsO(0.0,0.0);   //the location of origin is stored in an object of class coordinates using a member function called setcords(float x,float y)   
  c[1].setcordsO(3.0,4.5);   //location of destination:-can be anything, is stored in another object of class coordinates
  c[2].setcordsO(6.0,9.0);
  c[3].setcordsO(9.0,13.5); //setting the final location as initial location to check error.
  //setting the first destination coordinates.
  destX=c[1].retXO();// use (destLong-iLong)*111692.84 if working with degrees
  destY=c[1].retYO();//(destLat-iLat)*111692.84;
     
  X=globalX=0;  //initializing globalX and globalY
  Y=globalY=0;
  //--------------GENERAL SETUP ends-----------------------   

  
  PCICR  |= (1 << PCIE1);                     // Configure pin change interrupts for PCINT1
  PCMSK1 |= (1 << PCINT8);                    // Pin change interrupt for A0
  PCMSK1 |= (1 << PCINT9);                    // Pin change interrupt for A1
  PCMSK1 |= (1 << PCINT10);                   // Pin change interrupt for A2
  PCMSK1 |= (1 << PCINT11);

}

byte cycle = 1;

void loop()
{
  now = micros();
  if(cycle==pow(2,0)||cycle==pow(2,4))
  {
    I2Cdev::writeByte(0x68,0x37,0x02); //ask MPU9150 to let me talk to the magnetometer.(XCL,XDA) 
    compute_All();//1000us
    I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
    if((mh>20.0&&mh<160.0)||(mh>200.0&&mh<340.0)) //range of angles where the magnetometer is reliable against the external interference from the motor.
    {
      mh = 0.99*mh + 0.01*(tilt_Compensate(roll/57.3, pitch/57.3)); //tilt compensation takes angles in radians
    }//435us
    localization();//805
  }//2240us plenty of time here
  
  if(cycle==pow(2,3)||cycle==pow(2,7))
  {
    compute_All();//1000us
    I2Cdev::readBytes(0x0C, 0x03, 6, buf); //read the mag data
    m[1] = (((uint16_t)buf[0]) << 8) | buf[1]; // again, switching over X and Y axis readings because the mag has it's X aligned with accel's 
    m[0] = (((uint16_t)buf[2]) << 8) | buf[3]; // Y axis and vice-versa, which makes less sense to me. This is a personal preference thing and not mandatory
    m[2] = (((uint16_t)buf[4]) << 8) | buf[5]; //it just makes the debugging easy for me as stuff takes less time to make sense in my head.
    for(i=0;i<3;i++) //process mag data
    {
      M[i] = m[i];
      M[i] -= offsetM[i];
    } //135us
    localization();//805us
    Compute_Steering_Distance(); //525us
  }//2465us
  if(cycle==pow(2,1)||cycle==pow(2,5))
  {
    compute_All(); //1000us
    localization(); //805us
    Compute_Steering_Distance(); //525us
  }//2330us some gap here
  if(cycle==pow(2,2))
  {
    compute_All(); //1000us
    localization(); //805us
    if(a[0]==0&&a[1]==0&&a[2]==0&&g[0]==0&&g[1]==0&&g[2]==0) //12us to detect!
    {
      connection = false;
    }
    else
    {
      connection = true;
    }
    
  }//~1805us we got plenty of time in here
  if(cycle==pow(2,6))
  {
    compute_All(); //1000us
    localization(); //805us
    Compute_Steering_Distance(); //525us
    driver();//100us
  }//2430us
  cycle <<= 1;
  if(cycle==0)
  {
    cycle=1;
  }
  while(micros()-now<=2500); //fixing loop time.
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
