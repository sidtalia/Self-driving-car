#define CALIBERATION 0.00038        
#define loopFreq 400 //change this when cycle time is finalized  

void location_Update()
{
  //Ha is the Horizontal Acceleration.
  Ha=( (A[1]-(9.8*my_sin(pitch*0.01745)))*my_cos(pitch*0.01745) ); //the car could be going up an incline or going downhill
  X+= (!accelRequired)*((my_cos(mh*0.01745)*(float(dy)*CALIBERATION)) + (my_sin(mh*0.01745)*(float(dx)*CALIBERATION))) + (accelRequired*V*dt*my_cos(mh*0.01745)); //steady state assumption applied for accelgyro. accelRequired variable
  Y+= (!accelRequired)*((my_sin(mh*0.01745)*(float(dy)*CALIBERATION)) + (my_cos(mh*0.01745)*(float(dx)*CALIBERATION))) + (accelRequired*V*dt*my_sin(mh*0.01745));  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
  V=((float(dy)*CALIBERATION)*loopFreq)+(accelRequired*(V+Ha*dt));                                  //dy/dt because i trust optical flow a lot
}//585us

void localization()    //correction suggestions from gps
{  
  //==========LOCALIZATION BEGINS=======================================
  location_Update(); // look above this function. 585
  if(tick)//if new GPS data was received
  {
//    globalX = (longitude - iLong)*111692.84f; //get gps position w.r.t. origin
//    globalY = (latitude - iLat)*111692.84f;
    globalX = (longitude - lastLong)*111692.84f + lastX;
    globalY = (latitude - lastLat)*111692.84f + lastY;
    lastLong = longitude;
    lastLat = latitude; 
    
    globalX = gpsOpFlowKalman(globalX,Hdop,X,surfaceQuality); //run kalman filter on
    globalY = gpsOpFlowKalman(globalY,Hdop,Y,surfaceQuality); //gps coords and optical flow estimates. look into math_functions tab.
    
    X = lastX = globalX; //update previous estimates  
    Y = lastY = globalY; 

    tick = false; //make tick false so that it is known that the info from the gps is now old
  }

  globalX = X; //transfer values of estimates to global values every cycle
  globalY = Y; //because the angle and distance are calculated umy_sing these variables
  //----------LOCALIZATION ENDS-------------------------------------
}//800us
/*
void Compute_Steering_Distance()
{ 
  //----------CORRECTIONS USING THE LOCALIZATION RESULTS------------
  ml=anglecalcy(globalX,destX,globalY,destY);  //angle of line connecting bot to destination w.r.t E-W axis(X axis). Look into math functions tab. 

  if( mod(ml-mh)<=mod(ml-mh-360) ) //figuring out which angle is smaller, clockwise one or the anti-clockwise one.
  {
    correction = ml-mh;                            
  }
  else
  {
    correction = ml-mh-360;
  }
}//525us
*/
