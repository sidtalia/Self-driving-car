# Self-driving-car-STM-32 (just the readme. the project is currently private).
(Note: The project is still in progress and there are many things yet to be implemented. I also do need to improve the documentation so please cut me some slack if I missed out on something :P ).
This project is the third iteration of my attempt at a mini self-driving car. It started as an imitation of the ArduRover project without all the fancy GUI. I started doing this practically on a dare, but quickly realized I absolutely enjoyed the point where cars met robotics. Personally, I am a petrolhead, but I also love making things work autonomously and this project feels like the perfect spot in the middle.
![image](https://user-images.githubusercontent.com/24889667/64060910-eec52d80-cbf0-11e9-99f2-20f1574e10d9.png)
The first, second and third (current iteration). The first one had the connections hot-glued instead of soldered (yes). It has a come a long way since then.

The first version was built on a single Arduino Uno, the second on a pro-mini and the third one on an STM32F103C8T6 (aka the 'blue pill'). In all these projects, my aim has been to keep the cost of the controller and hardware as low as possible. This does put a limitation on how much processing power I can have but that's really where all the fun is; to send a rocket to the moon with computational power less than that of a modern wristwatch.

Oh and by the way, its rear wheel drive, which only makes the control problem harder.

Roadmap:
- [x] Outdoor relative localization (or odometry as most people like to call it)
- [x] Implement manual, partial manual and auto modes with failsafes
- [x] Trajectory generation and following
- [x] Preemptive braking to allow for faster speeds (Tested upto 8m/s on worn out tires. Will test again with new tires (normal medium soft compound)
- [x] Offline trajectory optimization given a set of waypoints (x,y) with a margin of adjustment and track width. 
- [ ] SONAR/LIDAR based obstacle avoidance for emergency braking (Mostly dealing with interference here. Should be dealt with soon).
- [ ] Landmark based localization using computer vision (Jevois A33)
- [ ] Opponent detection using Object matching (may or may not be faster than object detection) (Jevois A33)
- [ ] Realtime trajectory update for overtaking
- [ ] Designing overtaking policies.

### About the project
The car can race around a track given a set of waypoints (already optimized for minimum laptimes). An offline optimizer does the job of optimization (Test Codes folder, waypoint.py). It can also act as a low level controller for higher level agents. The car can be given a point (X,Y,theta) relative to it's current location. Ideally, the higher level agent should give waypoints that do not force the car to go through an obstacle (that's kind of the point of having a higher level agent). The lower level controller takes care of figuring out the throttle and steering control for getting to a particular point.

### Odometry and localization
The localization is supposed to happen in open outdoor environments using a filtered GPS data. I make the assumption that there is no exploitable feature in the environment. This is mostly because a system dependent on discernable features in the environment would not work as well in environments where there are none. The user can always add their own way of localization and simply send the location data to the low level controller (the support for this will be incorporated soon). Most projects like these should be tested in wide open spaces for the sake of safety and so GPS based localization made the most sense as it is the most "generic" way of localization.

![image](https://user-images.githubusercontent.com/24889667/64061985-13280680-cbff-11e9-98cb-fa1b4d33b29a.png)

The odometry is obtained by fusing data from GPS, optical flow, IMU+compass and also from a simplistic model of the car's propulsion system, although the last method is kept as a last-resort under multiple sensor failures. The state estimator also exploits the non-holonomic constraints. Using multiple sources of information allows the car to operate with a high degree of accuracy (long term) when all the sensors are operating in peak condition and with an acceptable level of accuracy when one or two of the sensors are not in peak condition. The odometry assumes that the car is moving on a flat, horizontal plane (fair assumption for a car that has a ride height of 15 mm and is unlikely to go off-roading).

Test for localization accuracy: https://youtu.be/GbBbyxaOqpI

Research paper for the same: https://ieeexplore.ieee.org/abstract/document/8979000
If this work is useful to you, please cite it! I'm a budding researcher and every citation goes a long way! :)

### Waypoint generation:
Currently you'll need to enter the x,y locations of the cones and corresponding initial guesses for the waypoints (because some cones are on the inside and some on the outside of the turn so I don't really have a way to automate the initial guesses at the moment) and the track width into the waypoint_test.py program. It will then produce a set of waypoints that produce a minimum curvature trajectory.

![image](https://user-images.githubusercontent.com/24889667/70389062-666cda80-19e0-11ea-997c-940d2b90f7f5.png)

The next step is to make this less dependent on human input, like the work done here: https://github.com/a1k0n/cycloid (shoutout to a1k0n). 

### Control
The control is based on bezier curve(3rd order) based trajectory generation. Code for offline trajectory optimization is in Testing right now (Test codes). There is also support for pre-emptive braking (more on that later) as a replacement for a real time velocity profile generator.

The speed control uses an asymmetric non-linear controller. Big words? Here's a simpler explanation:
1) The car does not speed up and slow down in the same way; the response of the brakes is different from the response of the throttle, therefore there is asymmetry in the process being controlled.
2) The car does not speed up by the same amount for the same amount of change in throttle at different speeds, hence there is a non-linearity in the process being controlled.

(Note: I know that a system with a non-linear response is also by default asymmetric in response, but as I have specified above, the system is asymmetric beyond the general understanding of what it means to be asymmetric. The throttle controls the power to the wheels while the brake controls the force, in essence the system controls 2 completely different variables, leading to an asymmetric response.)

This means that in order to control the speed, I'd need a controller that compensates for both these problems and hence the big-words.
The steering control is an open + closed loop progressive P controller (progressive as in the gain increases with the error, like in a progressive spring). The combination of these 2 controllers allows the car to remain under control for the most part even without preemptive braking.

The control system can update the friction coefficient of the surface if it detects that the car is sliding when turning, and if it does enter a slide, it can handle itself ;)
![GifMaker_20190831155907193](https://user-images.githubusercontent.com/24889667/64062747-533fb700-cc08-11e9-962e-d4252096618a.gif)

The system is also capable of preemptive braking, i.e., slowing down before the turn(as done in racing) instead of slowing down while turning.
Video link for preemptive braking :  https://www.youtube.com/watch?v=Ko5H_G4eCLo 
#### how preemptive braking is implemented: 
The car has a trajectory, it then finds the point of maximum curvature along that trajectory (upto the next checkpoint or waypoint) and determines maximum allowable speed for that curvature with some margin. The car then determines whether it should start slowing down for that point or not, on the basis of the deceleration required to hit that speed at that point. A more detailed discussion was done here (scroll down to the 5th last post which includes hand written notes): https://github.com/a1k0n/cycloid/pull/3

### Vision component: 
I have just started with computer vision please wait a couple of years uwu.

### Ground control system and V2V communications:
The previous iterations did not have a GCS. This made debugging extremely hard as there wasn't an option of data recording or viewing the internal state of the controller in real time. The GCS in this work has the bare minimum features needed for debugging and is not on par with GCS's like Missionplanner or Qgroundcontrol, however, it gets the job done. The GCS allows me to record data that I could use for debugging as well as for finding flaws in the system. It can plot the position data from the car in real time.

The car is equipped with an Xbee pro (2.4GHz) and so is the GCS (GCS here can be your laptop). The actual purpose of the Xbee was not just for communication with the GCS for debugging and monitoring, but rather to allow multiple agents to communicate with each other, meaning that the car is V2X ready from the hardware's point of view. I got the inspiration to do this from a project I did last year under Celestini Program India at IIT-Delhi where I worked on ADAS coupled with V2V communication: https://github.com/Celestini-Lucifer/ADAS

# Note for potential users : 
The car will not operate without a GCS by default. This is for safety purposes and not for the sake of functionality. If the car were to operate without a GCS connection and only be in control of the user via the on board Radio control, there would be a single point of failure in communications. Adding the GCS-compulsion gives the system 2 independent points of failure. This also compulsorily limits the range of operation (which depends on the kind of transceiver used. For xbee pros, this range is ~80 meters, which is more than enough for doing experiments with a 1/10 scale car.)

The system has multiple failsafes (note that the car can be operated in auto modes without the transmitter, but it is advised to always keep a transmitter with you as a failsafe).
1) If the car goes outside the range of the GCS (Xbee, ~80 meters) the car will come to a halt. If the user wants to recover it, they have to move a 3 position switch on their Flysky Transmitter to the middle position for manual control (refer to the hardware folder for BOM). 
2) If the main controller (stm32f103c8t6) fails and stops producing appropriate pwm signals for the esc and servo, the secondary controller will take over and relay the control signals coming from the receiver directly to the esc and motor.
3) If the user feels that the main controller is misbehaving or not working as intended, they can flip switch D on their flysky fs-i6 (switch D on channel 6) to force the secondary controller to relay control signals directly from the receiver to the esc and motor.


## Photos of this project : 
![img_20181223_232752](https://user-images.githubusercontent.com/24889667/51115201-90983580-182d-11e9-9a05-9175e0551990.jpg)

![img_20190114_182337](https://user-images.githubusercontent.com/24889667/51115209-968e1680-182d-11e9-9db0-57a545443a52.jpg)


## Building the project (in progress): 

(will update this as I build this project)

For the hardware specifications (including wiring specifications), 3d print files as well as 2 d drawings, go into the "hardware folder". 

For the back end libraries, go into the "libraries folder".

1) You will need : Arduino IDE (1.6.13 or better)

2) You will have to download the hardware files from : https://drive.google.com/file/d/1j8or7khmo2Z-QlrW-FHwhAybPbwVp9Ex/view?usp=sharing (its a slightly modified version of the original fork, I don't actually remember what changes I made but somehow the code after compilation takes 5 kB less memory) and unzip/extract it inside the "Arduino/hardware folder. You will also need to install the cortex M3 SAM board package in arduino, which can be installed easily through the boards manager in arduino IDE. Use this video for reference, the step of cloning the STM32 repository is replaced by downloading the zip and the rest remains the same : https://www.youtube.com/watch?v=MLEQk73zJoU&t=295s


The 'libraries folder' needs to go inside Arduino/libraries. The ino files need to go to wherever you store your sketches. Place the GCS folder where you keep your python files. The Baphomet code is for an Arduino Pro-Mini (5V, 16Mhz) and the Lucifer code is for the STM32F103c8t6. In the upload settings, set the CPU speed to overclocked (128Mhz) and upload method to serial.

### additional steps for Linux
Assuming you already have a functional installation of arduino (see here: https://www.arduino.cc/en/guide/linux )
You will probably face something that looks like this the first time you try to upload :
```
...../hardware/Arduino_STM32-master/tools/linux/serial_upload : error 13: Permission denied
```
just execute the following:
```
cd path/to/hardware/Arduino_STM32-master/tools/linux
sudo chmod +x *
```
and from within the same directory (path/to/hardware/Arduino_STM32-master/tools/linux)
```
cd stm32flash
sudo chmod +x *
```
now restart the arduino IDE and try uploading the code


### Common upload error:
```
Failed to init device.
stm32flash Arduino_STM32_0.9

http://github.com/rogerclarkmelbourne/arduino_stm32

Using Parser : Raw BINARY
Interface serial_posix: 115200 8E1

An error occurred while uploading the sketch
```
This happens when you forget to reset the stm32f103c8t6 (you have to push the reset button before the upload).
Again, watch this video for initial steps for stm32 bluepill https://www.youtube.com/watch?v=MLEQk73zJoU&t=295s

3) You will need Python
Python dependencies : 
Matplotlib,numpy, pyserial,scikit-learn :
```
pip3 install matplotlib
pip3 install numpy 
pip3 install pyserial
pip3 install scikit-learn
```
