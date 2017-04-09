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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //calling imu for values
  A[0]=ax;
  A[1]=ay;
  A[2]=az;
  G[0]=gx;
  G[1]=gy;
  G[2]=gz;
  for(i=0;i<3;i++)
  {
    A[i]-=offsetA[i]; //subtracting offset
    A[i]/=1638.4;     //mapping to real world values in m/s^2
    A[i]*=depress(A[i],1); //suppressing dead noise, suppression factor changed to 1 from 0.5 on 1/4/17
    
    A[i]= (0.8*A[i])+(0.2*lastA[i]); //applying the steady state assumption that the Acceleration can't change too much within 10-20 ms 
    lastA[i]=A[i]; //last A[i] ko bhi to value do ge na ,btw, gyro me ye assumptions kam nai karti and zaroorat bhi nai hai because gyro is way more stable.
    
    G[i]-=offsetG[i]; // subtracting offset
    G[i]/=131;        //mapping to degrees per second
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
  T[0]= (1-k)*T[0]+k*57.3*asin(A[1]/9.8);
  T[1]= (1-k)*T[1]-k*57.3*asin(A[0]/9.8);
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
  X+= (!accelRequired)*((cos(mh/57.3)*(dy/2000.0f)) + (sin(mh/57.3)*(dx/2000.0f))) + (accelRequired*V[1]*dt*cos(mh/57.3)); //steady state assumption applied for accelgyro. accelRequired variable
  Y+= (!accelRequired)*((sin(mh/57.3)*(dy/2000.0f)) + (cos(mh/57.3)*(dx/2000.0f))) + (accelRequired*V[1]*dt*sin(mh/57.3));  // acquires value =1 when optical flow experiences buffer overflow or surfacequality is lower than 25 
  V[1]=float(float(dy/2000.0f)/float(dt))+(accelRequired*(V[1]+Ha*dt));                                  //dy/dt because i trust optical flow a lot
}





