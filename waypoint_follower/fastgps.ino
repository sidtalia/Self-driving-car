const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };  //header of the incoming signal

struct NAV_POSLLH    //structure in which all the data will be stored
{
  unsigned char cls;  //class
  unsigned char id;   //id
  unsigned short len;  //length of packet
  unsigned long iTOW;   //time in microseconds since first fix
  long lon;              //longitude 
  long lat;               //latitude
  long height;          //height
  long hMSL;            //height above mean sea level
  unsigned long hAcc;     //horizontal accuracy 
  unsigned long vAcc;     //vertical accuracy 
};

NAV_POSLLH posllh;   //object of structure NAV_POSLLH
 
void calcChecksum(unsigned char* CK)    //function to calculate expected checksum 
{   
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) 
  {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS()    //bool function to tell us whether all the data has come in or not
{
  static int fpos = 0;                //variable to keep a track of where we are in the structure
  static unsigned char checksum[2];       //variables for storing checksum given in the message
  const int payloadSize = sizeof(NAV_POSLLH);     //size of payload

  while ( Serial.available() ) //while there is something on the UART/serial port. 
  {
    byte c = Serial.read();  //put data read from serial port into c
    if ( fpos < 2 )              //checking for first 2 bits of data 
    {
      if ( c == UBX_HEADER[fpos] )  //if the data in the first 2 bits matches up with the known standard first 2 bits, increment fpos
        fpos++;
      else
        fpos = 0;
    }
    
    else                            //when fpos>2 
    {
      if ( (fpos-2) < payloadSize )                   //when the first 2 bits of data have been read   
      {  
        ((unsigned char*)(&posllh))[fpos-2] = c;  //put the next bits of data into the object posllh of structure NAV_POSLLH by simply incrementing the position at which the data 
      }                                           //is being written rather than mentioning the name of the data etc    
      fpos++;                                     //i feel this should be fpos+=sizeof(c) because this didn't run properly in turbo c++
                                                  //it could be possible that in arduino IDE, incrementing the position implicitly means going to the next variable space.
                                                    
      if ( fpos == (payloadSize+2) )  //when payload has been read, calculate the expected checksum 
      {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) //compare our checksum with the given checksum
      {
        if ( c != checksum[0] )
          fpos = 0;                   //if checksum fails, the data can't be trusted, hence, move fpos back to 0
      }
      else if ( fpos == (payloadSize+4) )   // compare given checksum with our checksum
      {
        fpos = 0;
        if ( c == checksum[1] ) 
        {
          return true;     //data is valid, return true to indicate that we should now look at the data 
        }
      }
      else if ( fpos > (payloadSize+4) )  //when the entire sentence has been parsed, move fpos back to 0 
      {
        fpos = 0;
      }
    }
  }
  return false;
}
inline void updategps()  //make sure that you have while(!processGPS()){} before calling this if you are relying on an update from this function
{
    longitude=posllh.lon/10000000.0f;
    latitude=posllh.lat/10000000.0f;
    Hdop=posllh.hAcc/1000.0f;
}
