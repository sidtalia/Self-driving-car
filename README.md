## Self-driving-car (actually just a way point follower :P)  
waypoint following (obstacle avoidance unit is being re-developed as of now). video link:https://youtu.be/PnjM10glXuw 

This project is in essence supposed to be a lower level platform for developers to build on top of, something like an ardu rover of sorts but in my opinion better. 

# HARDWARE : 

My platform : HPI sprint 2 chassis (slightly modified).

Microcontroller : Arduino Pro mini.

Sensors used for localisation :

GPS (ublox LEA-6H)

IMU (MPU9250)

Optical flow sensor : ADNS3080 

(The optical flow sensor allows for very accurate positioning. However, it is only accurate upto a particular speed (which is usually 1/4th the theoretical maximum measureable speed for the sensor). It is therefore necessary to use the GPS and IMU for localisation at higher speeds (and it makes sense to use them at higher speeds because the faster you're accelerating greater the signal to noise ratio will be for the IMU).

Other miscellaneous hardware :

ADNS3080 needs a lot of light to work properly, therefore an LED from a computer mouse was used. The sensor was put in a shroud to do away with the shadow of the sensor itself. 

The motor has a steel casing around it to act as a magnetic short circuit to reduce low frequency interference.

![image](https://user-images.githubusercontent.com/24889667/44615607-bbbcc100-a85b-11e8-8619-28bebc285414.png)


# Motivation:
The reason why it has been built on an arduino mini pro and not an ARM(or some other fancy) controller is :

1)People that are new to robotics are gonna be more comfortable with arduino as compared to ARM(or some other fancy microcontroller), hence it would be easier for them to understand the code even if they are new to this field.

2)Arduino's are too cheap to bill.

3)(this is a personal one). IF NASA CAN SEND A ROCKET TO THE MOON USING COMPUTATIONAL POWER LESS THAN THAT OF A HANDHELD CALCULATOR I CAN MAKE A SELF DRIVING (actually just a waypoint follower but self driving car is more catchy) CAR ON AN ARDUINO MINI PRO. 

Why this is better than an ardu-rover: 
The ardu rover uses a point and shoot method for waypoint following, i.e., it simply takes the angle between where the car's nose is pointing and where the next waypoint is as the error and corrects that error using a PID controller on the steering. This works fine if you only have 2 points in your mission, the starting point and the ending point. This method does not take into account the position of the waypoint after the current one. 

In this project, I originally started out by copying what ardu-rover was doing but only to understand the basics of localization through sensor fusion and a little bit about control. I then incorporated trajectory planning using bezier curves. This allows the car to see the waypoints in a more "global" fashion rather than in the local fashion as is seen by the car in the ardu-rover. By seeing the points in a more global fashion, the car can calculate the appropriate trajectory that would allow the car to seamlessly travel through the waypoints. The trajectory planning also opens up the possibility of special manuevers like lane changing and parking.

As of now, the project is under development and I would advise against copying it and running it for yourself( I mean you could but don't blame me if something goes wrong).

# HOW IT WORKS (A layman explanation) : 
in the void setup:

1)setup all sensors, steering servo and motor esc.

2)take all points provided by the user and find the appropriate heading at each point(its the average of the slopes that join that point to the next point and the previous point)  

in void loop:

1)Find it's position and orientation (using the GPS, optical flow and IMU) 

2)Calculate the trajectory(bezier curve parameters) that seamlessly joins the current position and the next waypoint. 

2)find the position of the point where it will be by the end of the next to next cycle.

3)find the ROC at that point 

4)find the steering angle and maximum speed for the car at that point.

5)implement the steering angle and the speed through the steering servo and the esc that controls the driving motor.

# EXAMPLE CASE:
here is an example of what the controller "sees" 
![image](https://user-images.githubusercontent.com/24889667/44615527-9c249900-a859-11e8-86aa-40ae496153d2.png)

The white circles represent the waypoints that the user (me) provide to the arduino. The arduino then figures out the appropriate headings at each point. The appropriate heading at each point is (for now) the average of the angle made by the line joining that point to the last point and the line joining that point to the next point. 
at the beginning of each cycle major cycle, the arduino calculates where the car will be by the end of the next major cycle. Using this we can back calculate the parameter 't' for the bezier curve(taken as the ratio of (distance to the point at the end of next major cycle)/total distance to next waypoint
The red curve is the calculated trajectory(the arduino doesn't exactly calculate the trajectory, it directly jumps to the ROC at each point, The red trajectory drawn here(done in c++) is just to help the reader understand what is going on inside the arduino). The yellow curves represent the corresponding steering angle at every point.

To be clear, the arduino does not produce these images! This image was generated using a c++ program that ran on a laptop and is only for the sake of explanation. 



