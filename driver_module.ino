//the reason why I am using the servo library here is because the servo command just gets queued up and is executed in the background, 
//hence the time taken for the function to execute is reduced significantly. I can live with this because the control frequency can't be more than
//50Hz (if i exceed this i risk heating up the steering servo).
int throttle,steering;
#define maxValue 2000
#define minValue 1000
#define STEERINGNULL 1480
#define THROTTLENULL 1620
#define throttleKp 20
#define throttleKd 10
#define throttleKi 1 

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
  if(d<0.5)
  {    
    point++;
    if(point>=4)
    {
      throttle = 1400;  //if within 1 meter of target, stahp
    }
    else
    {
      destY=c[point].retYO();
      destX=c[point].retXO();
    }
  }
  else
  {
    error = 1 - V;
    throttle = (THROTTLENULL + capper(throttleKp*error - throttleKd*Ha + throttleKi*Serror));
    Serror += (error);
    Serror = capper(Serror);
  }
  if(input[0]<1500||!connection)
  {
    throttle = 1400;
    motor.writeMicroseconds(throttle);
    while(1);
  }
  steering = limiter(STEERINGNULL + 4*correction - 0.25*yawRate); //PD control on the steering.
  motor.writeMicroseconds(throttle); 
  steer.writeMicroseconds(steering);
}//100us

