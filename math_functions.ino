
inline float anglecalcy(float x1,float x2,float y1,float y2)  //everything is inline because fuck you thats why
{
  float angle = 57.3*atan2((y2-y1),(x2-x1));
  if(angle<0)
  {
    angle += 360;
  }
  return angle;
}//215us

inline float distancecalcy(float y1,float y2,float x1,float x2,int i)
{
  float delX = (x2-x1);
  float delY = (y2-y1);
  delX *= delX;
  delY *= delY;
  if(i==1)
  {
    return  111692.84*sqrt(delX + delY);   //distance between 2 points
  }
  else
  {
    return sqrt(delX + delY);    //distance directly in meters when input is in meters
  }
}//100us for calculating in meters.

inline float mod(float a)  //taking mod of a number
{
  return sqrt(a*a);
}


inline float gpsOpFlowKalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate)   //used in localization tab
{
  estimateError=(50.0/float(trustInEstimate));     //biasing was changed to 30 on 1/4/17 (not the american date standard.)
 // estimateError*=estimateError; //squaring the estimate error because i want the trust to rise very quickly if surface quality is good and fall very quickly if it is lower than 20)
  KG=(estimateError/(estimateError+gpsError));
  return (KG*gpscord+(1-KG)*estimate);  
}//95us

inline float depress(float a,float k)               //used to depress accelgyro values
{
  return (a*a*a*a)/(k+a*a*a*a);
}

inline float my_asin(float a)
{
  return a*(1+(0.5*a*a)); //35us
}
inline float my_sin(float a)
{
  if(a>1.57&&a<4.71)
  {
    a = 3.14 - a;
  }
  if(a>=4.71)
  {
    a -= 6.28;
  }
  return a*(1-(a*a*0.1667)); //35us
}
inline float my_cos(float a)
{
  if(a>1.57&&a<4.71)
  {
    a = 3.14-a;
    return ((0.5*a*a)-1);
  }
  if(a>=4.17)
  {
    a -=6.28;
  }
  return (1-(0.5*a*a)); //25us
}

inline float my_tan(float x)
{
  float x2,ans; 
  if(x>1.57&&x<4.71)
  {
    x -= 3.14;
  }
  if(x>=4.71)
  {
    x -= 6.28;
  }
  x2 = x*x;
  ans = x*(1 + 0.333*x2 + 0.1333*x2*x2);
  if(x>1.45||x<-1.45)
  {
    ans *= ans;
  }
  if(x>1.55||x<-1.55)
  {
    ans *= ans;
  }
  if(x>1.56||x<-1.56)
  {
    ans *= ans;
  }
  return ans;
} //88us 

void intersection(float x1,float y1,float m1,float x2,float y2,float m2,float &x,float &y)
{
  float s1,s2,c1,c2,inv;
  s1 = my_tan(m1*0.017452);
  s2 = my_tan(m2*0.017452);
  c1 = y1 - s1*x1;
  c2 = y2 - s2*x2;
  inv = 1/(s2-s1);
  x = inv*(c1-c2);
  y = inv*((c1*s2) - (c2*s1));
}//316us

float ROC(float x0,float y0,float x1,float y1,float x2,float y2,float t)
{
  float Dx,Dy,D2x,D2y,r,d;
  D2x = (x2 - 2*x1 + x0);
  D2y = (y2 - 2*y1 + y0);
  Dx = D2x*t + x1 - x0;
  Dy = D2y*t + y1 - y0;
  r = (Dx*Dx) + (Dy*Dy);
  r *=(r*r); //r^3
  r = sqrt(r); //36 us to sqrt. r^1.5
  d = (Dx*D2y - Dy*D2x);
  if(d==0)
  {
    return 1000;
  }
  else
  {
    return r/d;
  }
}//224us

float steer_Angle(float roc)
{
  if(roc>500||roc<-500)
  {
    return 0;
  }
  return atan(0.25/roc);
}//230us


