

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
  float heading;
  float cosRoll = my_cos(roll); //putting the cos(roll) etc values into variables as these values are used over and 
  float sinRoll = my_sin(roll); //over, it would simply be a waste of time to keep on calculating them over and over again 
  float cosPitch = my_cos(pitch);//hence it is a better idea to just calculate them once and use the stored values.
  float sinPitch = my_sin(pitch);
  //the following formula is compensates for the pitch and roll of the object when using magnetometer reading. 
  float Xh = -M[0]*cosRoll + M[2]*sinRoll;
  float Yh = M[1]*cosPitch - M[0]*sinRoll*sinPitch + M[2]*cosRoll*sinPitch;
  
  Xh = Xh*0.2 + 0.8*magbuf[0]; //smoothing out the X readings
  magbuf[0] = Xh;

  Yh = Yh*0.2 + 0.8*magbuf[1]; //smoothing out the Y readings
  magbuf[1] = Yh;
  heading = 57.3*atan2(Yh,Xh);
  if(heading<0) //atan2 goes from -pi to pi 
  {
    return 360 + heading; //2pi - theta
  }
  return heading;
}//410us worst case 

void readMPU()   //function for reading MPU values. its about 80us faster than getMotion6() and hey every us counts!
{
  Wire.beginTransmission(0x68);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(0x68,14); //request 14 bytes from mpu
  //300us for all data to be received. 
  //each value in the mpu is stored in a "broken" form in 2 consecutive registers.(for example, acceleration along X axis has a high byte at 0x3B and low byte at 0x3C 
  //to get the actual value, all you have to do is shift the highbyte by 8 bits and bitwise add it to the low byte and you have your original value/. 
  a[0]=Wire.read()<<8|Wire.read();  
  a[1]=Wire.read()<<8|Wire.read(); 
  a[2]=Wire.read()<<8|Wire.read(); 
  g[0]=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  g[0]=Wire.read()<<8|Wire.read();  
  g[1]=Wire.read()<<8|Wire.read();
  g[2]=Wire.read()<<8|Wire.read();
}


void readAll()
{// while we wait for the mag to send us the values, we read the
  readMPU(); //291us
  for(int i=0;i<3;i++)
  {
    A[i] = a[i];
    A[i] -= offsetA[i];
    A[i] *= 0.0006103;
    A[i] = 0.8*A[i] + 0.2*lastA[i];
    lastA[i] = A[i];

    G[i] = g[i];
    G[i] -= offsetG[i];
    G[i] *= 0.030516;
    G[i] = 0.8*G[i] + 0.2*lastG[i];
    lastG[i] = G[i];
  }//243us
  updateOpticalFlow(); // 160us
  localizer(); // go into the GPS tab to see the inner workings. 30us
}//724us

void compute_All()
{ 
  readAll();//724us
  pitch += G[0]*dt;
  roll  += G[1]*dt; 
  mh += G[2]*dt;  
  if(A[1]<2&&A[1]>-2)
  {
    pitch = 0.99*pitch + 0.01*(57.3*my_asin(A[1]*0.102)); //0.102 = 1/9.8
  }
  if(A[0]<2&&A[0]>-2)
  {
    roll  = 0.99*roll  - 0.01*(57.3*my_asin(A[0]*0.102)); //using the accelerometer to correct the roll and pitch.
  }
  mh += G[0]*dt*my_sin(-roll/57.3); //compensates for pitch and roll of gyro itself (roll pitch compensation to the yaw).
  if(mh >= 360.0) // the mh must be within [0.0,360.0]
  {
    mh -= 360.0;
  }
  if(mh < 0)
  {
    mh += 360;
  }
  yawRate = G[2]; // yaw rate is probably used for steering PID.
}//1000us worst case 
