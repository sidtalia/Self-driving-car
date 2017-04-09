
//every function is inline because there are so many functions that the overhead time of each function will add up and increase the control loop time significantly.  
//and also because space isn't a big deal here(YET).

void localizer() //function to figure out our original location. not inline because it is called only once
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //keep pinging the accelgyro so that it doesn't go to sleep 
  if(counter<1)      //first coordinates are taken as initial longitude and latitude to compare next data with
  {
    while(!processGPS())
    {
      delay(50);
    }
    updategps();   //update coordinates
    iLat=latitude;   // initial coordinates
    iLong=longitude; 
    check--;   //reduce check 
  }
  if(counter>=1)  //ilat and ilong have been set 
  {
    while(!processGPS())  //this is to ensure that we get data from gps
    {
      delay(50);
    }
    
    updategps();  //update lat,lon,lastlat,lastlon
    
    if((latitude-lastLat)*111692.84f>0.01||(latitude-lastLat)*111692.84f<(-0.01))   //Zeroing in on the actual position
    {
      iLat=latitude; //whenever the new coordinates are too far away from previous origin, we change our origin to the new origin assuming that the new ones are more accurate
      check++; 
    }
       
    if((longitude-lastLong)*111692.84f>0.01||(longitude-lastLong)*111692.84f<(-0.01))
    {
      iLong=longitude;
      check++; 
    }
    
    check--;  //reduce check   
  }
  counter++;     //increment counter
  delay(200);  //delay because gps updates at a rate of 5Hz 
}


inline void gpsSuggest()   //correction suggestions from gps
{  
  //============FIGURING OUT INITIAL LOCATION=======================
  if(counter==0)    //when gpsSuggest() is called for the first time 
  {
      while(check>0)  //call the localizer function 1200 times to figure out initial coordinates (iLat,iLong)
     {
      localizer();
     }
     c[0].setcords(iLat,iLong);   //the location of origin is stored in an object of class coordinates using a member function called setcords(float x,float y)   
     c[1].setcords(iLat,iLong-0.00003);   //location of destination:-can be anything, is stored in another object of class coordinates
     
     c[2].setcordsO(3.0,0.0);
     c[3].setcordsO(0.0,0.0);
     
     destLong=c[1].retx();    //current destination coordinates 
     destLat=c[1].rety(); //this may not be there in the final code, a flag might be used to change the destination coords in case there are multiple checkpoints
     destX=(destLong-iLong)*111692.84;
     destY=(destLat-iLat)*111692.84;
     
     globalX=0;  //initializing globalX and globalY
     globalY=0;
     raw=compass.readRaw();  
     mh=57.3*atan2(raw.YAxis,-raw.XAxis);  //using magnetometer to figure out initial heading

     X=globalX;   //initializing estimates as global values
     Y=globalY;
  
     m=mh;    
     delay(1000);  
  }  
  //----------DONE FIGURING OUT INITIAL LOCATION AND HEADING------------

  //==========LOCALIZATION BEGINS=======================================
  
  updateOpticalFlow();
  callimu();  //use the imu to figure out new location as well as current heading.
  callSonar();
  
  if( processGPS())  //processGPS is in the fastGPS tab,if new(true) data has come in then, 
  { 
    updategps();    //update the lat,lon,lastlat,lastlon
    globalX=(longitude-iLong)*111692.84f;
    globalY=(latitude-iLat)*111692.84f;
    
    globalX=gpsOpFlowKalman(globalX,(posllh.hAcc/1000.0f),X,surfaceQuality);    //calculate distance moved-psuedo kalman
    globalY=gpsOpFlowKalman(globalY,(posllh.hAcc/1000.0f),Y,surfaceQuality);
    X=globalX;      //new X coordinates for optical flow to update are the results from the kalman filter
    Y=globalY;

    theta=mod(mh-m);    //angular deviation from last time
   
    if(mod(theta)<0.5)
    {
      V[1]=(1-KG)*V[1] + 5.0*KG*(distancecalcy(latitude,lastLat,longitude,lastLong,1));  //computer kya jane sin(0)/(0) ka swad.
    }
    
    else
    {
      V[1]= (1-KG)*V[1] + 5.0*KG*theta*distancecalcy(latitude,lastLat,longitude,lastLong,1)/(2*sin(theta/114.6)); // second part is the expected max speed 
    }
  
    tick=1; 
    m=mh;  //last heading is now current heading
    
  }  
  
  if(tick==0)  //if gps update was just performed, then don't update global variables as that has been done already,else do it 
  {
    globalX=X;
    globalY=Y;
  }
  else
  {
    tick=0;  //reset tick to 0 for next cycle
  }
  //----------LOCALIZATION ENDS-------------------------------------
  
  //----------CORRECTIONS USING THE LOCALIZATION RESULTS------------
  ml=anglecalcy(globalX,destX,globalY,destY);  //angle of line connecting bot to destination w.r.t E-W axis(X axis)
  correction=mh-ml;                            
  d=distancecalcy(globalY,destY,globalX,destX,0); //returns distance in meters
}




