// this tab contains functions that aid in collecting data
//compute_All is the main function in this tab and it collects accelgyro data, optical Flow data and gps data while it waits for the mag data,
//thus saving time in communication.
void readMagDuringSetup() // a separate function is used during setup() because in the looping mode 
{                         // there is no 10ms delay after enabling the magnetometer, that 10ms of free time is used to read other sensors.
  I2Cdev::writeByte(0x68,0x37,0x02);
  delay(1);
  I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2Cdev::readBytes(0x0C, 0x03, 6, buf); // get 6 bytes of data
  m[1] = (((uint16_t)buf[0]) << 8) | buf[1]; // the mag has the X axis where the accelero has it's Y and vice-versa
  m[0] = (((uint16_t)buf[2]) << 8) | buf[3]; // so I just do this switch over so that the math appears easier to me. 
  m[2] = (((uint16_t)buf[4]) << 8) | buf[5]; // I prefer to have a standardized sense of X and Y instead of each sensor having it's own separate X and Y.

  for(i=0;i<3;i++)
  {
    M[i] = m[i];
    M[i] -= offsetM[i];
    M[i] /= 300;//is this even needed? do i really need to convert the readings to gauss?
  }
}

float tilt_Compensate(float roll,float pitch) //function to compensate the magnetometer readings for the pitch and the roll.
{
  float cosRoll = cos(roll); //putting the cos(roll) etc values into variables as these values are used over and 
  float sinRoll = sin(roll); //over, it would simply be a waste of time to keep on calculating them over and over again 
  float cosPitch = cos(pitch);//hence it is a better idea to just calculate them once and use the stored values.
  float sinPitch = sin(pitch);
  //the following formula is compensates for the pitch and roll of the object when using magnetometer reading. 
  float Xh = -M[0]*cosRoll + M[2]*sinRoll;
  float Yh = M[1]*cosPitch - M[0]*sinRoll*sinPitch + M[2]*cosRoll*sinPitch;
  
  Xh = Xh*0.2 + 0.8*magbuf[0]; //smoothing out the X readings
  magbuf[0] = Xh;

  Yh = Yh*0.2 + 0.8*magbuf[1]; //smoothing out the Y readings
  magbuf[1] = Yh;
  if(atan2(Yh,Xh)<0) //atan2 goes from -pi to pi 
  {
    return 57.3*(6.28 + atan2(Yh,Xh)); //2pi - theta
  }
  return 57.3*atan2(Yh,Xh);
}


void readAll()
{
  long timer=0;  
  I2Cdev::writeByte(0x68,0x37,0x02); //ask MPU9150 to let me talk to the magnetometer.(XCL,XDA) 
  delay(1);
  I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer, it will now take ~7ms to get the readings, to be on the safe side, i wait for 10ms

  timer = millis();
// while we wait for the mag to send us the values, we read the accel and gyro
  accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]); //get the a/g values. takes about ~500us
  for(int i=0;i<3;i++)
  {
    A[i] = a[i];
    A[i] -= offsetA[i];
    A[i] *= 0.0006103;
    A[i] = 0.8*A[i] + 0.2*lastA[i];
    lastA[i] = A[i];

    G[i] = g[i];
    G[i] -= offsetG[i];
    G[i] *= 0.007633;
    G[i] = 0.8*G[i] + 0.2*lastG[i];
    lastG[i] = G[i];
  }
  //update dy,dx and surfaceQuality of the optical flow 
  updateOpticalFlow(); // not sure how long this actually takes but i m pretty sure it is pretty darn quick.
  //and get our new position(if available) from GPS 
  localizer(); // go into the GPS tab to see the inner workings.
               //it updates the longitude,latitude(in degrees) and Hdop(in m) and sets a variable "tick" as true 
               //which lets the "compute_Steering_distance function know if new gps data has arrived. 
  while(millis()-timer<10); //stuck until 10 milliseconds pass by
 
  I2Cdev::readBytes(0x0C, 0x03, 6, buf); //read the mag data
  m[1] = (((uint16_t)buf[0]) << 8) | buf[1]; // again, switching over X and Y axis readings because the mag has it's X aligned with accel's 
  m[0] = (((uint16_t)buf[2]) << 8) | buf[3]; // Y axis and vice-versa, which makes less sense to me. This is a personal preference thing and not mandatory
  m[2] = (((uint16_t)buf[4]) << 8) | buf[5]; //it just makes the debugging easy for me as stuff takes less time to make sense in my head.

  for(i=0;i<3;i++) //process mag data
  {
    M[i] = m[i];
    M[i] -= offsetM[i];
    M[i] /= 300; // again, do i need to do this? not that it's a huge inconvenience but still.
  }
}


void compute_All()
{
  readAll(); // read data from all sensors. takes ~12ms.
  
  pitch += G[0]*dt;
  roll  += G[1]*dt; 
  mh += G[2]*dt;  
  T[0] +=G[0]*dt; //rotation around gyro's X axis, this is not necessarily the same as the pitch.
  T[1] +=G[1]*dt; //rotation around gyro's Y axis, this is not necessarily the same as the roll. Doing this for the roll-pitch rate compensation to the yaw.
  pitch = 0.99*pitch + 0.01*(57.3*asin(A[1]*0.102)); //0.102 = 1/9.8
  roll  = 0.99*roll  - 0.01*(57.3*asin(A[0]*0.102)); //using the accelerometer to correct the roll and pitch.
  mh += G[0]*dt*sin(-T[1]/57.3); //compensates for pitch and roll of gyro itself (roll pitch compensation to the yaw).
  if(mh >= 360.0) // the mh must be within [0.0,360.0]
  {
    mh -= 360.0;
  }
  if(mh < 0)
  {
    mh += 360;
  }
  if((mh>20.0&&mh<160.0)||(mh>200.0&&mh<340.0)) //range of angles where the magnetometer is reliable against the external interference from the motor.
  { //complimentary filter on gyro and mag when the values are in the range where interference along the Y axis makes little to no difference.
    mh = 0.8*mh + 0.2*(tilt_Compensate(roll/57.3, pitch/57.3)); //tilt compensation takes angles in radians
  }
  yawRate = G[2]; // yaw rate is used for steering PID.
}
