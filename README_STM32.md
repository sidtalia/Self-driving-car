# Self-driving-car-STM-32
This project is the third iteration of my attempt at a mini self-driving car. It started as an imitation of the ArduRover project without all the fancy GUI. I started doing this practically on a dare, but quickly realized I absolutely enjoyed the point where cars met robotics. Personally, I am a petrolhead, but I also love making things work autonomously and this project feels like the perfect spot in the middle.
![image](https://user-images.githubusercontent.com/24889667/64060910-eec52d80-cbf0-11e9-99f2-20f1574e10d9.png)
The first, second and third (current iteration). The first one had the connections hot-glued instead of soldered (yes). It has a come a long way since then.

The first version was built on a single Arduino Uno, the second on a pro-mini and the third one on an STM32F103C8T6 (aka the 'blue pill'). In all these projects, my aim has been to keep the cost of the controller and hardware as low as possible. This does put a limitation on how much processing power I can have but that's really where all the fun is; to send a rocket to the moon with computational power less than that of a modern wristwatch.

### About the project
This project is supposed to act as a lower level controller for higher level agents. The car can be given a point (X,Y,theta) relative to it's current location. The car can also reactively avoid obstacles if ultrasonic sensors are present in the build (this will be replaced by lidars in the future). Ideally, the higher level agent should give waypoints that do not force the car to go through an obstacle (that's kind of the point of having a higher level agent). The lower level controller takes care of figuring out the throttle and steering control for getting to a particular point.

### Odometry and localization
The localization is supposed to happen in open outdoor environments using a filtered GPS data. I make the assumption that there is no exploitable feature in the environment. This is mostly because a system dependent on discernable features in the environment would not work as well in environments where there are none. The user can always add their own way of localization and simply send the location data to the low level controller (the support for this will be incorporated soon). Most projects like these should be tested in wide open spaces for the sake of safety and so GPS based localization made the most sense as it is the most "generic" way of localization.

![image](https://user-images.githubusercontent.com/24889667/64061985-13280680-cbff-11e9-98cb-fa1b4d33b29a.png)

The odometry is obtained by fusing data from GPS, optical flow, IMU+compass and also from a simplistic model of the car's propulsion system, although the last method is kept as a last-resort under multiple sensor failures. The state estimator also exploits the non-holonomic constraints. Using multiple sources of information allows the car to operate with a high degree of accuracy (long term) when all the sensors are operating in peak condition and with an acceptable level of accuracy when one or two of the sensors are not in peak condition. The odometry assumes that the car is moving on a flat, horizontal plane (fair assumption for a car that has a ride height of 15 mm and is unlikely to go off-roading).

Test for localization accuracy: https://youtu.be/GbBbyxaOqpI


### Control
The control is based on bezier curve(3rd order) based trajectory generation. At the moment, the generated trajectory is not an optimized one. There is however, support for pre-emptive braking (more on that later).

The speed control uses an asymmetric non-linear controller. Big words? Here's a simpler explanation:
1) The car does not speed up and slow down in the same way; the response of the brakes is different from the response of the throttle, therefore there is asymmetry in the process being controlled.
2) The car does not speed up by the same amount for the same amount of change in throttle at different speeds, hence there is a non-linearity in the process being controlled.

This means that in order to control the speed, I'd need a controller that compensates for both these problems and hence the big-words.
The steering control is an open + closed loop progressive P controller (progressive as in the gain increases with the error, like in a progressive spring). The combination of these 2 controllers allows the car to remain under control for the most part even without preemptive braking.

The control system can update the friction coefficient of the surface if it detects that the car is sliding when turning, and if it does enter a slide, it can handle itself ;)
![GifMaker_20190831155907193](https://user-images.githubusercontent.com/24889667/64062747-533fb700-cc08-11e9-962e-d4252096618a.gif)

The system is also capable of preemptive braking, i.e., slowing down before the turn(as done in racing) instead of slowing down while turning.
Video link for preemptive braking :  https://www.youtube.com/watch?v=Ko5H_G4eCLo 
#### how preemptive braking is implemented: 
The car has a trajectory, it then finds the point of maximum curvature along that trajectory (upto the next checkpoint or waypoint) and determines maximum allowable speed for that curvature with some margin. The car then determines whether it should start slowing down for that point or not, on the basis of the deceleration required to hit that speed at that point. A more detailed discussion was done here (scroll down to the 5th last post which includes hand written notes): https://github.com/a1k0n/cycloid/pull/3

For my project, since the bezier curve trajectory is dynamic, the location of the maximas can change and so I would need to find them in real time. However, the problem with this is that analytically finding the maximas is impossible (requires solving for the roots of a fifth order polynomial) and finding them numerically would take too much time for a reasonable level of error (+/- 1% )
![image](https://user-images.githubusercontent.com/24889667/64473489-3a458180-d185-11e9-83cf-b4f9081d4508.png)

This evaluation takes about 3 milliseconds on my 3.6GHz 64 bit laptop. The loop time of the entire code has to be less than 2.5 milliseconds on a 128MHz 32 bit microcontroller, so this appears to be a really bad approach

To deal with this inconvenience, I came up with a "magic formula" that generates a rough guess which is within 10% of the true value and then I just use 2 iterations of the newton-rhapson method to zone in on the true value. This gets me the result (both maximas) in about 400 microseconds on the microcontroller. Furthermore, the time complexity of this process is equal to the time complexity of newton raphson method which is in the log(n) family.


### Ground control system and V2V communications:
The previous iterations did not have a GCS. This made debugging extremely hard as there wasn't an option of data recording or viewing the internal state of the controller in real time. The GCS in this work has the bare minimum features needed for debugging and is not on par with GCS like Missionplanner or even Qgroundcontrol, however, it gets the job done. The GCS allows me to record data that I could use for debugging as well as for finding flaws in the system. It can plot the position data from the car in real time.

The car is equipped with an Xbee pro (2.4GHz) and so is the GCS (GCS here can be your laptop). The actual purpose of the Xbee was not just for communication with the GCS for debugging and monitoring, but rather to allow multiple agents to communicate with each other, meaning that the car is V2X ready from the hardware's point of view. I got the inspiration to do this from a project I did last year under Celestini Program India at IIT-Delhi where I worked on ADAS coupled with V2V communication: https://github.com/Celestini-Lucifer/ADAS

# Note for potential users : 
The car will not operate without a GCS by default. This is for safety purposes and not for the sake of functionality. If the car were to operate without a GCS connection and only be in control of the user via the on board Radio control, there would be a single point of failure in communications. Adding the GCS-compulsion gives the system 2 independent points of failure. This also compulsorily limits the range of operation (which depends on the kind of transceiver used. For xbee pros, this range is ~80 meters, which is more than enough for doing experiments with a 1/10 scale car.)


## Photos of this project : 
![img_20181223_232752](https://user-images.githubusercontent.com/24889667/51115201-90983580-182d-11e9-9a05-9175e0551990.jpg)

![img_20190114_182337](https://user-images.githubusercontent.com/24889667/51115209-968e1680-182d-11e9-9db0-57a545443a52.jpg)


## Building the project (in progress): 

(will update this as I build this project)

For the hardware specifications and 3d print files as well as 2 d drawings, go into the "hardware folder"(currently unavailable). 

For the back end libraries, go into the "libraries folder"(currently unavailable).

1) You will need : Arduino IDE (1.6.13 or better)

2) You will have to download the hardware files from : https://drive.google.com/file/d/1j8or7khmo2Z-QlrW-FHwhAybPbwVp9Ex/view?usp=sharing (its a slightly modified version of the original fork, I don't actually remember what changes I made but somehow the code after compilation takes 5 kB less memory) and unzip/extract it inside the "Arduino/hardware folder. You will also need to install the cortex M3 SAMD board package in arduino, which can be installed easily through the boards manager in arduino IDE. Use this video for reference, the step of cloning the STM32 repository is replaced by downloading the zip and the rest remains the same : https://www.youtube.com/watch?v=MLEQk73zJoU&t=295s

3) You will need Python 3 (it may or may not work on python 2, I have only tried it with python 3.5 and 3.6.5).
Python dependencies : 
Matplotlib,numpy, pyserial,scikit-learn :
```
pip3 install matplotlib
pip3 install numpy 
pip3 install pyserial
pip3 install scikit-learn
```
