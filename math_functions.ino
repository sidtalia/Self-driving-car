
float anglecalcy(float x1,float x2,float y1,float y2)  //everything is inline because fuck you thats why
{
  float angle = 57.3*atan2((y2-y1),(x2-x1));
  if(angle<0)
  {
    angle += 360;
  }
  return angle;
}//215us

float distancecalcy(float y1,float y2,float x1,float x2,int i)
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

inline __attribute__((always_inline)) float mod(float a)  //taking mod of a number
{
  if(a<0)
  {
    return -a;
  }
  return a;
}

//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
inline __attribute__((always_inline)) float gpsOpFlowKalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate)   //used in localization tab
{
  estimateError=(50.0/float(trustInEstimate));     //biasing was changed to 30 on 1/4/17 (not the american date standard.)
  estimateError*=estimateError; //squaring the estimate error because i want the trust to rise very quickly if surface quality is good and fall very quickly if it is lower than 20)
  KG=(estimateError/(estimateError+gpsError));
  return (KG*gpscord+(1-KG)*estimate);  
}//95us

//inline __attribute__((always_inline)) float depress(float a,float k)               //used to depress accelgyro values
//{
//  return (a*a*a*a)/(k+a*a*a*a);
//}

inline __attribute__((always_inline)) float my_asin(float a)
{
  return a*(1+(0.5*a*a)); //35us
}
inline __attribute__((always_inline)) float my_sin(float a)
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
inline __attribute__((always_inline)) float my_cos(float a)
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

//float my_tan(float x)
//{
//  float x2,ans; 
//  if(x>1.57&&x<4.71)
//  {
//    x -= 3.14;
//  }
//  if(x>=4.71)
//  {
//    x -= 6.28;
//  }
//  x2 = x*x;
//  ans = x*(1 + 0.333*x2 + 0.1333*x2*x2);
//  if(x>1.45||x<-1.45)
//  {
//    ans *= ans;
//  }
//  if(x>1.55||x<-1.55)
//  {
//    ans *= ans;
//  }
//  if(x>1.56||x<-1.56)
//  {
//    ans *= ans;
//  }
//  return ans;
//} //88us 

float get_T(float V,float d)
{
  float parameter = 0.04*V/d; //distance covered in 2 cycles/total distance.
  if(parameter>1)
  {
    return 1;
  }
  return parameter;
}

inline __attribute__((always_inline)) void Curvature(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t,float max_Acceleration)
{
  float KX1,KX2,KX3,KY1,KY2,KY3;
  float C,Expected_YR;
  float delX,delY,del2X,del2Y;
  float denominator;
  KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3; //60
  KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3; //60
  KX2 = 6*X1 - 12*X2 + 6*X3; //40
  KY2 = 6*Y1 - 12*Y2 + 6*Y3; //40
  KX3 = 3*(X2 - X1); //15
  KY3 = 3*(Y2 - Y1); //15
  delX = t*t*KX1 + t*KX2 + KX3; //40
  delY = t*t*KY1 + t*KY2 + KY3; //40
  del2X = 2*t*KX1 + KX2; //25
  del2Y = 2*t*KY1 + KY2; //25
  denominator = delX*delX + delY*delY; //25
  denominator *= denominator*denominator; //20
  denominator = sqrt(denominator); //35
  C = ((delX*del2Y) - (delY*del2X)); //25
  C /= denominator; //30
  
  correction = 14.6115*C; //10 . atan of small values is almost the same as the value.(tan(x) = x for small x) 
  Expected_YR = 57.3*V*C; // V is velocity, C is 1/ROC, 57.3->takes radians/s to deg/s 
  yaw_Compensation = Expected_YR - yawRate;//15us
  C = mod(C); // happens in practically no time. ~4us
  if(C<0.01)
  {
    C=0.01;
  }
  Vmax = sqrt(max_Acceleration/C); //60us.  
} //585us.

inline __attribute__((always_inline)) void get_Intermediate_Points(float slope1,float slope2,float X1,float X2,float Y1,float Y2,float d)
{
  int1[0] = X1 + 0.4*my_cos(slope1*0.01745)*d; // 60
  int1[1] = Y1 + 0.4*my_sin(slope1*0.01745)*d; //70
  int2[0] = X2 - 0.4*my_cos(slope2*0.01745)*d; //60
  int2[1] = Y2 - 0.4*my_sin(slope2*0.01745)*d; //70
}//260us

inline __attribute__((always_inline)) void generate_Slopes() 
{
  for(int i = 1;i<n-1;i++)
  {
    c[i].slope = (anglecalcy( c[i-1].retXO(),c[i].retXO(),c[i-1].retYO(),c[i].retYO() )+anglecalcy( c[i].retXO(),c[i+1].retXO(),c[i].retYO(),c[i+1].retYO() ))*0.5;
  }
}
