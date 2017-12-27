//the reason why I am writing my own code to write the PWM instead of using the servo library is because 
//It is not capable of writing the signals simultaneously, the signals go one after another. That method would have a worst case
// time of 2*number of servos/esc's which is not a good thing if for example i intend to use separate in wheel motors to control the speed and direction.

long esc_timer,end_timer;
long throttle,steering;
#define maxValue 2000
#define minValue 1000
#define STEERINGNULL 1480
#define THROTTLENULL 1640

inline void servoWrite()
{
  while(PORTD>=8)
  {
    end_timer = micros(); //note the time

    if(end_timer>=throttle) //if time has exceeded or is equal to the value of throttle
    {
      PORTD &= MOTOR_PIN; //pull the corresponding pin low
    }
    if(end_timer>=steering)
    {
      PORTD &= STEERING_PIN;
    }
  }
}


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

inline void driver() // function to operate the servo and esc.
{
  esc_timer = micros();//note the time 
  PORTD |= PULL_HIGH; //pull the pins corresponding to the esc and servo high
  if(d<2)
  {
    point++;
    if(point>=4)
    {
      throttle = 1400 + esc_timer ;  //if within 1 meter of target, stahp
    }
    else
    {
      destY=c[point].retYO();
      destX=c[point].retXO();
    }
  }
  else
  {
    throttle = (THROTTLENULL + 50 - retard) + esc_timer;
  }
  steering = limiter(STEERINGNULL + 16*correction - yawRate) + esc_timer; //PD control on the steering.
  servoWrite();
}



