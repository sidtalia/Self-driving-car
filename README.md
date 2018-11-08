## Self-driving-car (actually just a way point follower :P)  
the project is no longer being maintained. I have switched over to STM32F103C8T6. The new project is over at Self-driving-car-STM-32.
(obstacle avoidance unit is being re-developed as of now). video link for this (old) version 

Test1 :https://youtu.be/PnjM10glXuw 

Test2 : https://www.youtube.com/watch?v=IqyNvaG0JE0

(Refer to the image at the bottom of readme). The localisation is not vision based. it uses GPS, IMU and optical flow (ADNS3080 for localisation). The reason for using optical flow is that it gives much better resolution compared to wheel encoders plus you can also measure sideways movement! 

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

# EXAMPLE CASE:
here is an example of what the controller "sees" 
![image](https://user-images.githubusercontent.com/24889667/44615527-9c249900-a859-11e8-86aa-40ae496153d2.png)

The white circles represent the waypoints that the user (me) provide to the arduino. The arduino then figures out the appropriate headings at each point. The appropriate heading at each point is (for now) the average of the angle made by the line joining that point to the last point and the line joining that point to the next point. 
at the beginning of each cycle major cycle, the arduino calculates where the car will be by the end of the next major cycle. Using this we can back calculate the parameter 't' for the bezier curve(taken as the ratio of (distance to the point at the end of next major cycle)/total distance to next waypoint
The red curve is the calculated trajectory(the arduino doesn't exactly calculate the trajectory, it directly jumps to the ROC at each point, The red trajectory drawn here(done in c++) is just to help the reader understand what is going on inside the arduino). The yellow curves represent the corresponding steering angle at every point.

To be clear, the arduino does not produce these images(that would be a bit too computationally expensive)! This image was generated using a c++ program that ran on a laptop and is only for the sake of explanation. The arduino only knows of the coordinates of the waypoints. No data is dumped on to the arduino about the appropriate trajectory. The arduino finds the trajectory on the basis of it's current position, heading and the target waypoint, heading in real time, i.e., the trajectory is not pre-calculated. This means that if by chance the car was to go off-track a bit(due to loss of grip for whatever reason), it would still be following an optimum trajectory(Well it's not optimum right now but it will be optimum in the future iterations when i shift this project to something that has a clock speed of more than 16MHz).

# HOW IT WORKS (A layman explanation) : 
in the void setup:

1)setup all sensors, steering servo and motor esc.

2)take all points provided by the user and find the appropriate heading at each point(its the average of the slopes that join that point to the next point and the previous point)  

in void loop:

1)Find it's position and orientation (using the GPS, optical flow and IMU) 

2)Calculate the trajectory(bezier curve parameters) that seamlessly joins the current position and the next waypoint. 

3)find the position of the point where it will be by the end of the next to next cycle.

4)find the ROC at that point 

5)find the steering angle and maximum speed for the car at that point.

6)implement the steering angle and the speed through the steering servo and the esc that controls the driving motor.

# HOW IT WORKS (A bit more in-depth):
setup : 
remains mostly the same.

loop :
As the arduino pro mini has very limited processing speed, it is not possible to have an observation and control frequency of 400Hz. Therefore, the observation frequency is kept at 400Hz while the control frequency is kept at 50 Hz (also because servos will overheat above a control frequency of 50Hz)

The main idea is : we do the positioning in each cycle, but divide the other miscellaneous tasks across multiple cycles (kind of like RTOS but not really). 

for example : A 20ms cycle is divided into 8 sub-cycles. In each sub cycle the arduino finds the position. After that, it can do a sub-task related to trajectory planning and then end the sub-cycle. In the next sub-cycle, it finds the position and then executes the next task related to trajectory planning and so on. 

1)Finding position : 
  
  get heading, yawRate, roll and pitch from IMU.
  
  get measured speed from optical Flow sensor
  
  if optical Flow data is reliable (good surface features):
  
    find and remove biases and errors in the accelerometer to improve accelerometer estimated speed (basically tune the accelerometer)
  
  if optical Flow data is unreliable (bad surface features):
    
    use the "tuned" accelerometer to estimate speed //Its assumed that optical flow is initially reliable 
    
  using the velocity estimate and the heading estimate, predict the position
  
  if new gps data is available:
    fuse the new gps data with the position estimate (its a psuedo kalman filter).
 
 2)Calculating trajectory : 
 
  Find the distance between waypoint and the car. Also find the distance that the car will travel within the next 2 full cycles (2 full   cycles because the control signal i send to the servo and esc will be implemented only by the end of the 2nd full cycle. This gives the system a lead to compensate for the lag introduced by the mechanics. It can be improved by knowing the bode-plot for the system but this works fine because the change in the control inputs will not be drastic (its an assumption)
  
  Use that distance to find the "t" parameter for the bezier curve.
  
  in the next sub-cycle, find the intermediate points for the bezier curve. find the ROC(there is no need to calculate the entire trajectory just to find the ROC of a point, the ROC can be found directly because math ) of the point corresponding to the "t" parameter just found.
  
3) driver code : from the ROC, compute the steering angle (this section requires knowledge of the wheelbase of the car and it's maximum steering angle) and the maximum speed at which the car can travel, assuming no acceleration while turning(bad assumption, I know). 

The driver code takes into account the net horizontal acceleration felt by the car. The closer it is to the maximum set limit, the more the car will back off from either the throttle or braking(if the car is already accelerating, it will back-off from the throttle, if the car is already braking, it will back off from braking).

The speed control implements an open+closed loop control on the speed. the open loop control assumes a linear relation between speed and esc input. The open loop controller more or less sets a "baseline" for the closed loop controller. This allows me to get rid of the I term from the PID controller (more or less) (i know its a bad assumption but hey its a start!) 
for the braking, it uses a closed loop controller (since controlling deceleration does not require an open loop assumptions).

The steering control also implements an open+closed loop control on the steering. The open loop control would be to directly convert the required steering to the equivalent PWM signal and the closed loop control compensates the error between desired and measured yaw rate. If the car isn't turning as fast as we want, give more steering (this assumes we are not understeering). If the car is turning faster than we want(most likely oversteering) then reduce the steering. This automatically makes sure that if the car were to enter a slide , it would still remain more or less in control (this is how the drift boxes in rc drift cars work btw).

and I guess thats about it! after this its just writing the signals to the servo and esc.

There are a lot of miscellaneous functions in the code that are used to make sure that the arduino doesn't demand the hardware to do something it can't, as well as fail-safe measures.
  
  
  

