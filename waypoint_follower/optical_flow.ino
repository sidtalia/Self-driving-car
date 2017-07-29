
inline void  updateOpticalFlow(void) 
{
  // Read sensor
  uint8_t buf[4];
  spiRead(ADNS3080_MOTION_BURST, buf, 4);
  uint8_t motion = buf[0];
  //Serial.print(motion & 0x01); // Resolution
  if (motion & 0x01) 
  {  
    dx = buf[1];   //caliberation for conversion to meters.   
    dy = buf[2];
    surfaceQuality = buf[3];
    accelRequired= false;
    if(surfaceQuality<20)
    {
      surfaceQuality = 1;
      accelRequired  = true ; //use accelerometer 
    }
  }
  else if(motion & 0x10)  //buffer overflow
  {
    accelRequired = true ;
  }
}

inline void reset(void)              //reset. used almost never
{
  digitalWrite(RESET_PIN, HIGH); // Set high
  delayMicroseconds(10);
  digitalWrite(RESET_PIN, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
}


void spiWrite(uint8_t reg, uint8_t data)     //function writes the data in the register of specified address . used once
{
  spiWrite(reg, &data, 1);
}

void spiWrite(uint8_t reg, uint8_t *data, uint8_t length) 
{
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg | 0x80); // Indicate write operation
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

inline uint8_t spiRead(uint8_t reg)   //function returns the data in the buffer stored at the address passed 
{
  uint8_t buf;
  spiRead(reg, &buf, 1);
  return buf;
}

inline void spiRead(uint8_t reg, uint8_t *data, uint8_t length) 
{
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);          //telling the optical flow sensor that we want to read it (MISO mode)

  SPI.transfer(reg); // Send register address
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  memset(data, 0, length); // Make sure data buffer is 0
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

