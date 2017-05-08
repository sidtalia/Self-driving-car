
//every function is inline because there are so many functions that the overhead time of each function will add up and increase the control loop time significantly.  
//and also because space isn't a big deal here(YET).

void localizer() //function to figure out our original location. not inline because it is called only once
{
  accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);  //keep pinging the accelgyro so that it doesn't go to sleep 
  if(processGPS())
  {
    updategps();
  }
 
}

inline void gpsSuggest()   //correction suggestions from gps
{  
  //==========LOCALIZATION BEGINS=======================================
  updateOpticalFlow();
  callimu();  //use the imu to figure out new location as well as current heading. driver is called inside this function 
  
  if( processGPS())  //processGPS is in the fastGPS tab,if new(true) data has come in then, 
  { 
    updategps();    //update the lat,lon,lastlat,lastlon
    globalX=(longitude-iLong)*111692.84f;
    globalY=(latitude-iLat)*111692.84f;
    
    globalX=gpsOpFlowKalman(globalX,Hdop,X,surfaceQuality);    //calculate distance moved-psuedo kalman
    globalY=gpsOpFlowKalman(globalY,Hdop,Y,surfaceQuality);
    X=globalX;      //new X coordinates for optical flow to update are the results from the kalman filter
    Y=globalY;
    
    tick=true;
  }  
  
  if(!tick)  //if gps update was just performed, then don't update global variables as that has been done already,else do it 
  {
    globalX=X;
    globalY=Y;
  }
  else
  {
    tick=false;  //reset tick to 0 for next cycle
  }
  //----------LOCALIZATION ENDS-------------------------------------
  
  //----------CORRECTIONS USING THE LOCALIZATION RESULTS------------
  ml=anglecalcy(globalX,destX,globalY,destY);  //angle of line connecting bot to destination w.r.t E-W axis(X axis)
  correction=mh-ml;                            
  d=distancecalcy(globalY,destY,globalX,destX,0); //returns distance in meters
}

