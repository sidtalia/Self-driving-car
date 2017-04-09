inline void callSonar()
{
  call_slave(SONAR_MODULE_ADDRESS);  //call the slave and get the correction and retard values 
  
  sonarCorrection = message[1]-100;                   //decrypt sonarCorrection
  //sonarCorrection*=depress(sonarCorrection,256);    //depress sonarCorrection to remove dead noise if any
  sonarCorrection*=SONAR_GAIN;                      //multiply with proportionality constant
  
  retard=message[0];
  retard*=depress(retard,16000);        // remove dead noise
  retard*=RETARD_GAIN;                 // multiply with proportioanlity gain
}

