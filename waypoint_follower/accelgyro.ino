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
  T[0]= 0.99*T[0]-0.573*asin(A[1]*0.102);
  T[1]= 0.99*T[1]+0.573*asin(A[0]*0.102);
}

inline void headingUpdate()
{
  mh+=(G[2]*dt);   //update the heading
  raw=compass.readRaw();
  mh=0.9*mh+(5.73*atan2(raw.YAxis,-raw.XAxis));  //applying complimentary filter on (10%)compass and (90%)gyro
}

inline void locationUpdate()
{
  Ha=((A[1]-(9.8*sin(T[0]*0.01745)))/cos(T[0]*0.01745)); //the car could be going up an incline or going downhill
  Ha*=(depress(Ha,0.5));//choti choti values gand marae
  X+= (!accelRequired)*((cos(mh*0.01745)*(dy*0.0005)) + (sin(mh*0.01745)*(dx*0.0005))) + (accelRequired*V*dt*cos(mh*0.01745)); //steady state assumption applied for accelgyro. accelRequired variable
  Y+= (!accelRequired)*((sin(mh*0.01745)*(dy*0.0005)) + (cos(mh*0.01745)*(dx*0.0005))) + (accelRequired*V*dt*sin(mh*0.01745));  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
  V=float(float(dy*0.0005)/float(dt))+(accelRequired*(V+Ha*dt));                                  //dy/dt because i trust optical flow a lot
}




