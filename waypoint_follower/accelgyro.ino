// every function is inline because fuck you thats why. 
/*
╔═══════╗╔═╗─╔═╗╔═╗─╔═╗╔══════╗
║███████║║█║─║█║║█║─║█║║██████║
║█╔╗█╔╗█║║█║─║█║║█║─║█║║█╔══╗█║
╚═╝║█║╚═╝║█╚═╝█║║█║─║█║║█║──╚═╝
───║█║───║█████║║█║─║█║║█║
───║█║───║█╔═╗█║║█║─║█║║█║╔═══╗
───║█║───║█║─║█║║█╚═╝█║║█╚╝█▀█║
───║█║───║█║─║█║║█████║║█▄▄▄▄█║
───╚═╝───╚═╝─╚═╝╚═════╝╚══════╝
╔═╗────╔═══╗╔════╗╔════╗
║█║────║███║║████║║████║
║█║────╚╣█╠╝║█╔══╝║█╔══╝
║█║─────║█║─║█╚══╗║█╚══╗
║█║─────║█║─║████║║████║
║█║─╔═╗─║█║─║█╔══╝║█╔══╝
║█╚═╝█║╔╣█╠╗║█║───║█╚══╗
║█████║║███║║█║───║████║
╚═════╝╚═══╝╚═╝───╚════╝ 
 */


inline void callimu()
{
  //-------EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA BEGINS--
  accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]); //calling imu for values
  driver();
  for(i=0;i<3;i++)
  {
  
    A[i]=a[i];
    A[i]-=offsetA[i]; //subtracting offset
    A[i]*=0.006103; //mapping to real world values in m/s^2 , instead of dividing by 1638.4 i multiply by 1/1638.4 to save on time as multiplication is faster 
    
    A[i]= (0.8*A[i])+(0.2*lastA[i]); //applying the steady state assumption that the Acceleration can't change too much within 2.5 ms 
    lastA[i]=A[i]; 

    G[i]=g[i];
    G[i]-=offsetG[i]; // subtracting offset
    G[i]*=0.007633;   //mapping to degrees per second by multiplying with 1/131 
    G[i]=(0.8*G[i])+(0.2*lastG[i]);  //buffer filter,same as that for accel.
    lastG[i]=G[i]; 
    G[i]*=depress(G[i],2); //suppressing dead noise
  }
  //-----EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA ENDS------
  
  orientationUpdate();  
  headingUpdate(); 
  locationUpdate(); 
}

inline void orientationUpdate()
{
  for(i=0;i<2;i++)
  {
    T[i]+=(G[i]*dt);  //T[0]=pitch,T[1]=roll,T[2]=yaw
  }
  k=sqrt(A[0]*A[0]+A[1]*A[1]+A[2]*A[2]);
  k=(19.6-k)*k/1960;
  T[0]= (1-k)*T[0]-k*57.3*asin(A[1]/9.8);
  T[1]= (1-k)*T[1]+k*57.3*asin(A[0]/9.8);
}

inline void headingUpdate()
{
  mh+=(G[2]*dt);   //update the heading
  raw=compass.readRaw();
  mh=0.9*mh+(5.73*atan2(raw.YAxis,-raw.XAxis));  //applying complimentary filter on (10%)compass and (90%)gyro
}

inline void locationUpdate()
{
  Ha=((A[1]-(9.8*sin(T[0]/57.3)))/cos(T[0]/57.3)); //the car could be going up an incline or going downhill
  Ha*=(depress(Ha,0.5));//choti choti values gand marae
  X+= (!accelRequired)*((cos(mh/57.3)*(dy/2000.0f)) + (sin(mh/57.3)*(dx/2000.0f))) + (accelRequired*V*dt*cos(mh/57.3)); //steady state assumption applied for accelgyro. accelRequired variable
  Y+= (!accelRequired)*((sin(mh/57.3)*(dy/2000.0f)) + (cos(mh/57.3)*(dx/2000.0f))) + (accelRequired*V*dt*sin(mh/57.3));  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
  V=float(float(dy/2000.0f)/float(dt))+(accelRequired*(V+Ha*dt));                                  //dy/dt because i trust optical flow a lot
}




