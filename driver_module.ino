
//the reason why I am using the servo library here is because the servo command just gets queued up and is executed in the background, 
//hence the time taken for the function to execute is reduced significantly. I can live with this because the control frequency can't be more than
//50Hz (if i exceed this i risk heating up the steering servo).
int throttle,steering;
#define maxValue 2000
#define minValue 1000
#define STEERINGNULL 1500
#define THROTTLENULL 1550
#define throttleKp 20
#define throttleKd 10
#define throttleKi 1
#define VMAX 17
#define SAFE_SPEED 3 
#define OPEN_GAIN 20
#define CLOSED_GAIN 10
#define BRAKE_GAIN 50
#define STEERING_GAIN 12.4

inline int limiter(int input) // prevent the values from going outside the 2000-1000us range
{
  if(input>maxValue)
  {
    return maxValue;
  }
  if(input<minValue)
  {
    return minValue;
  }
  return input;
}

float capper(float input)
{
  if(input>100)
  {
    return 100;
  }
  if(input<-100)
  {
    return -100;
  }
  return input;
}

inline void driver() // function to operate the servo and esc.
{
  float deceleration,backoff,resultant;
  float Ax,Ay;
  Ax = A[0]+9.8*my_sin(roll*0.01745);
  Ay = Ha;
  resultant = sqrt(Ha*Ha + Ax*Ax);
  if(input[0]>1700)
  {
    if(Vmax>VMAX)//if maximum possible speed is more than the speed limit
    {
     Vmax = VMAX; //speed setpoint is the speed limit.
    }
  } 
  else if(input[0]>1500&&input[0]<1700)
  {
    if(Vmax>SAFE_SPEED)
    {
      Vmax = SAFE_SPEED;
    }
  }
  else if(input[0]<1500&&input[0]>1200)
  {
    Vmax = 0;
  }
  error = Vmax - V; //speed setpoint - current speed
  if(error>=0)//Required velocity is greater than the current velocity
  {
    if(Ha>0)//if we are already speeding up
    {
      backoff = 20*(resultant - MAX_ACCELERATION);
      if(backoff<0)
      {
        backoff = 0;
      }
    }
    else
    {
      backoff = 0;
    }
    if(!accelRequired)
    {
      throttle = THROTTLENULL + OPEN_GAIN*Vmax + CLOSED_GAIN*error - backoff; //open loop + closed loop control for reducing error at a faster rate.
    }
    else if(accelRequired)
    {
      throttle = THROTTLENULL + OPEN_GAIN*Vmax - backoff;
    }
  }
  if(error<0)//required velocity is less than current velocity.
  {
    if(Ha<0) //if we are already slowing down
    {
      backoff = 20*(resultant - MAX_ACCELERATION);
      if(backoff<0)
      {
        backoff = 0;
      }
    }
    else
    {
      backoff = 0;
    }
    //max deceleration would be ~ 10m/s*s, 500/10 = 50, therefore open loop brake gain would be 50.
    deceleration = error*50 - Ha;//0.2 m/s change within 1/50th of a second would require 10m/s*s of deceleration
    //deceleration error is the difference between the deceleration required(error*50) and the deceleration we already have. 
    if(deceleration<-10) //prevent reset windup
    {
      deceleration = -10;
    }
    throttle = THROTTLENULL + BRAKE_GAIN*deceleration + backoff;
  }
  if(d<1)
  {    
    point++;
    if(point>=n)
    {
      point = 1;  //if within 1 meter of target, stahp
    }
    else // change dest coordinates 
    {
      destY=c[point].retYO();
      destX=c[point].retXO();
      destSlope = c[point].slope;
    }
  }
  //just making sure that we don't short the battery and shut down the controller
  if(throttle>last_Throttle+50)
  {
    throttle = last_Throttle +50;
  }
  last_Throttle = throttle;
  
  if(input[0]<1100||!connection) //failsafe. the thing that I write before anything else.
  {
    throttle = 1500;//this is to make sure the car doesn't go into reverse on startup if it doesn't find a transmitter/failsafe thingy
    motor.writeMicroseconds(throttle);
    delay(60);
    throttle = 1100;
    motor.writeMicroseconds(throttle);
    while(1);
  }
  steering = limiter(STEERINGNULL + STEERING_GAIN*correction + 0.4*yaw_Compensation); //open loop + closed loop control
  motor.writeMicroseconds(throttle); 
  steer.writeMicroseconds(steering);
}//140us
