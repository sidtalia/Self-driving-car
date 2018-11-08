#define CALIBERATION 0.00043       
#define loopFreq 400 //change this when cycle time is finalized  

void location_Update()
{
  float cosmh = my_cos(mh*0.01745);
  float sinmh = my_sin(mh*0.01745);
  float Vacc;
  float radPitch = 0.01745*pitch;
  float cosPitch = my_cos(radPitch);
  float sinPitch = my_sin(radPitch);
  //Ha is the Horizontal Acceleration.
  A[1] -= AccBias; //remove offset.
  Ha=( (A[1]-(9.8*sinPitch))/cosPitch ); //the car could be going up an incline or going downhill
  if(!accelRequired)
  {
    X += ( ( cosmh*float(dy) ) + ( sinmh*float(dx) ) )*CALIBERATION; //steady state assumption applied for accelgyro. accelRequired variable
    Y += ( ( sinmh*float(dy) ) + ( cosmh*float(dx) ) )*CALIBERATION;  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
    Vacc = V + Ha*dt; //Predicted velocity.
    V = float(dy)*CALIBERATION*loopFreq;//get measurement
    //find the difference between prediction and measurement. (no updates because optical flow measurement is accurate AF) 
    AccBias = (Vacc - V)*cosPitch;//keep adjusting bias while optical flow is trustworthy. cosPitch is to align it with the A[1]'s axis. 
  }
  else if(accelRequired||V>5)
  {
    X += V*dt*cosmh + sinmh*float(dx)*CALIBERATION;//sideways movement is still valid.
    Y += V*dt*sinmh + cosmh*float(dx)*CALIBERATION;
    V += Ha*dt; //predict velocity.
  }                                  
}
//271us
void localization()    //correction suggestions from gps
{  
  //==========LOCALIZATION BEGINS=======================================
  float KG_1;
  location_Update(); // look above this function. 505us
  if(tick)//if new GPS data was received
  {
    globalX = (longitude - lastLong)*111692.84f + lastX; //21us
    globalY = (latitude - lastLat)*111692.84f + lastY; //21us
    lastLong = longitude;
    lastLat = latitude; 

//    globalX = gpsOpFlowKalman(globalX,Hdop,X,surfaceQuality);
//    globalY = gpsOpFlowKalman(globalY,Hdop,Y,surfaceQuality);
    estimateError=(50.0/float(surfaceQuality)); //29us
    estimateError *= estimateError;
    KG=(estimateError/(estimateError+Hdop)); //36.5us
    KG_1 = 1-KG;//just saving time yo. trust me, it makes a world of difference.
    globalX = KG*globalX + KG_1*X; // 25us
    globalY = KG*globalY + KG_1*Y; // 25us
    
    X = lastX = globalX; //update previous estimates  
    Y = lastY = globalY; 

    tick = false; //make tick false so that it is known that the info from the gps is now old
  }//158us

  globalX = X; //transfer values of estimates to global values every cycle
  globalY = Y; //because the angle and distance are calculated using these variables
  //----------LOCALIZATION ENDS-------------------------------------
}//437us
