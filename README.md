# Self-driving-car
waypoint following (obstacle avoidance unit is being re-developed as of now).
This project is in essence supposed to be a lower level platform for developers to build on top of, something like an ardu rover of sorts but in my opinion better. 
The reason why it has been built on an arduino mini pro and not an ARM controller is :
1)People that are new to robotics are gonna be more comfortable with arduino as compared to ARM, hence it would be easier for them to understand the code even if they are new to this field.
2)Arduino's are too cheap to bill. 
3)(this is a personal one). IF NASA CAN SEND A ROCKET TO THE MOON USING COMPUTATIONAL POWER LESS THAN THAT OF A HANDHELD CALCULATOR I CAN MAKE A SELF DRIVING CAR ON AN ARDUINO MINI PRO. 

Why this is better than an ardu-rover: 
The ardu rover uses a point and shoot method for waypoint following, i.e., it simply takes the angle between where the car's nose is pointing and where the next waypoint is as the error and corrects that error using a PID controller on the steering(although a P/PI controller would actually work just fine if you don't exceed the limit of friction, something that the programmers of ardu-rover probably did not know, because who cares about actually analysing the system using physics right?). This works fine if you only have 2 points in your mission, the starting point and the ending point. This method does not take into account the position of the waypoint after the current one. 
In this project, I originally started out by copying what ardu-rover was doing but only to understand the basics of localization through sensor fusion and a little bit about control. I then incorporated trajectory planning using bezier curves. This allows the car see the waypoints in a more "global" fashion rather than in the local fashion as is seen by the car in the ardu-rover. By seeing the points in a more global fashion, the car can calculate the appropriate trajectory that would allow the car to seamlessly travel through the waypoints. The trajectory planning also opens up the possibility of special manuevers like lane changing(which will be incorporated much, much later, possibly 6 months from now, I am still ironing out some kinks in it) and parking(going in reverse while parking, knowing with absolute certainty what the path will be is important). 

As of now, the project is under development and I would advise against copying it and running it for yourself( I mean you could but don't blame me if something goes wrong). What I intend to add in the future is:
1)communication module/interface/ hack: since the one and only hardware UART is used by the GPS and I2C,SPI and analog voltage block the main code while in reception mode, I plan on using the analog pins(A0,A1,A2,A3,A6,A7) to receive information in a PWM format since that stuff can be read by interrupts and will not block the main code in reception mode. In this method, the transmitter would send a peice of information by breaking it into 4 bit peices. 4 bit = 0-15. the arduino can measure a minimum of 4us time difference in a pulse, therefore the pulse width would have to be about 60us plus some headroom(say 20us). These 4 bit peices of information are then put back together to get the original information ( 01101101 = 0110 + 1101 = 6 + 13 = (PWM) 24us pulse and a 52us pulse (plus some head room on each pulse). 
This communication interface can be used by modules, say an obstacle avoidance module, to provide suggestions for changes in the heading or application of the brakes. it can also be used by a computer vision based localization module to improve the absolute localization of the car.
2)Standard modules for the car: Once done with ironing out the kinks in the base project, I will be working on a standard obstacle avoidance module that uses MAXBOTIX SONAR sensors and a separate arduino to process their data. the unit would then send suggestions to the main controller via the communication interface(This can be done without the 4 bit nonsense) for the steering and the braking. Another module I will be developing would be an arduino that uses the "communication interface(1)" to send data to the main controller from the outside world(which could be a mavlink or a raspberry pi or pretty much anything), in essence acting like a translator. Another module that I am thinking of but not sure if I should include is a computer vision based localization module(The reason why I don't like the idea is because it would then require that the environment around it be modified, something that isn't practical if you wish to scale this thing, maybe I will use google's API for landmark detection and use that, but I doubt that any landmarks would be visible to the car in an urban canyon).

The way this car works (A layman explanation) : 
in the void setup:
1)setup all sensors, servos and escs
2)take all points and find the appropriate heading at each point(its the average of the slopes that join that point to the next point and the previous point)  

in void loop:
1)Find it's position and orientation (using the GPS, optical flow and IMU) 
2)Calculate the trajectory(bezier curve parameters) that seamlessly joins the current position and the next waypoint. 
2)find the position of the point where it will be by the end of the next to next cycle.
3)find the ROC at that point 
4)find the steering angle and maximum speed for the car at that point.
5)implement the steering angle and the speed through the steering servo and the esc that controls the driving motor.
