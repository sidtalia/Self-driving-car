
//every function is inline because there are so many functions that the overhead time of each function will add up and increase the control loop time significantly.  
//and also because space isn't a big deal here(YET).

inline void location_Update()
{
  //Ha is the Horizontal Acceleration.
  Ha=( (A[1]-(9.8*sin(pitch*0.01745)))*cos(pitch*0.01745) ); //the car could be going up an incline or going downhill
  Ha*=(depress(Ha,0.5));//suppressing small values of Ha to reduce the noise.
  X+= (!accelRequired)*((cos(mh*0.01745)*(float(dy)/2000.0f)) + (sin(mh*0.01745)*(float(dx)/2000.0f))) + (accelRequired*V*dt*cos(mh*0.01745)); //steady state assumption applied for accelgyro. accelRequired variable
  Y+= (!accelRequired)*((sin(mh*0.01745)*(float(dy)/2000.0f)) + (cos(mh*0.01745)*(float(dx)/2000.0f))) + (accelRequired*V*dt*sin(mh*0.01745));  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
  V=float((float(dy)/2000.0f)*50)+(accelRequired*(V+Ha*dt));                                  //dy/dt because i trust optical flow a lot
}


inline void Compute_Steering_Distance()   //correction suggestions from gps
{  
  //==========LOCALIZATION BEGINS=======================================
  compute_All();  //collect optical_flow,mag,accelgyro and gps data and compute heading,yaw rate,pitch,roll(look in data_collect tab)
  location_Update(); // look above this function.
  if(tick)//if new GPS data was received
  {
    globalX = (longitude - iLong)*111692.84f; //get gps position w.r.t. origin
    globalY = (latitude - iLat)*111692.84f;

    globalX = gpsOpFlowKalman(globalX,Hdop,X,surfaceQuality); //run kalman filter on
    globalY = gpsOpFlowKalman(globalY,Hdop,Y,surfaceQuality); //gps coords and optical flow estimates. look into math_functions tab.
    
    X = globalX; //update previous estimates  
    Y = globalY; 

    tick = false; //make tick false so that it is known that the info from the gps is now old
  }

  globalX = X; //transfer values of estimates to global values every cycle
  globalY = Y; //because the angle and distance are calculated using these variables
  //----------LOCALIZATION ENDS-------------------------------------
  
  //----------CORRECTIONS USING THE LOCALIZATION RESULTS------------
  ml=anglecalcy(globalX,destX,globalY,destY);  //angle of line connecting bot to destination w.r.t E-W axis(X axis). Look into math functions tab. 

  if( mod(ml-mh)<=mod(ml-mh-360) ) //figuring out which angle is smaller, clockwise one or the anti-clockwise one.
  {
    correction = ml-mh;                            
  }
  if(mod(ml-mh)>mod(ml-mh-360) )
  {
    correction = ml-mh-360;
  }
  d=distancecalcy(globalY,destY,globalX,destX,0); //returns distance in meters. look into math_functions tab.
}
