int L=50,C=50,R=50; //starting with non zero default values
int lastC=50;
#define ROOT3 1.732
#define THRESHOLD 200
#define BrakeP 1
#define BrakeD 1


void rebootSensors(){
  digitalWrite(MAXSONAR_VCC, LOW);                // Pull it low before starting up
  delay(10);                           // A little delay
  digitalWrite(MAXSONAR_VCC, HIGH);               // Power up the sensors.
  delay(250);                           // The sensors take 250ms to boot up.

  // Give an initial trigger and then enter high impedance state
  pinMode(MAXSONAR_TRIGGER,OUTPUT);               // Set the trigger pin to be output.
  digitalWrite(MAXSONAR_TRIGGER,HIGH);              // Trigger begins
  delayMicroseconds(100);                     // Trigger should be greater than 20us and less than 48ms
  digitalWrite(MAXSONAR_TRIGGER,LOW);               // Trigger ends
  pinMode(MAXSONAR_TRIGGER,INPUT);                // INPUT mode is high impedance for AVR controllers.
}



int msToCm(int microseconds) //function to convert time into distance  
{
  return microseconds/58; //launde ko physics ati hai
}

int read_Distance(int prev,int num)
{
  int D;
  D= (9*D + msToCm(input[num]) )/10; 
  D = D>THRESHOLD ? THRESHOLD : D ;
  return D;
}


void transferData()
{
  L= read_Distance(L,0);
  
  lastC=C;  //because braking has a PD system
  C= read_Distance(C,1);
  
  R= read_Distance(R,2);
}

inline void callSonar()
{
  transferData();
  sonarCorrection = depress( 57.3*atan(ROOT3*(L-R)/(min(L,R))),100);
  retard = BrakeP*(THRESHOLD-C)+ BrakeD*(lastC-C);  
}


