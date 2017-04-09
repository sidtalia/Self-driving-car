
void call_slave(int address)
{
     // begin transmission with usr slave arduino
  for(i=0;i<2;i++)
  {               // transmit to usr slave arduino
    Wire.beginTransmission(address);
    Wire.write(i);           
    Wire.endTransmission();
    
    Wire.requestFrom(address, 1);    // request 1 byte usr slave arduino
    
    while (Wire.available()) 
    {                           // slave may send less than requested
       message[i]= Wire.read(); // receive message 
    }
  }
  
}
