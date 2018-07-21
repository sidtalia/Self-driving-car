#STUFF NEEDED FOR RE-CREATING THE PROJECT:

1) Ackerman steering based car/rover:
    -> 1:10 scale or larger.
    
    -> preferably electric (nitro and gas engines are fussy. Brushed motor might be preferred due to their hard iron offset. However, brushless motors are more efficient and there would be no static discharges that could potentially harm the electronics
    
    -> preferably has longitudenal power train layout( the motor shaft is parallel to the longitudenal axis of the car ). However, even if the motor is perpendicular to the longitudenal direction, the code should run fine
    
    -> can be either belt of shaft driven. If working indoors and noise is a problem, prefer belt driven powertrain.
    
2) GPS: Preferably Ublox LEA-6H or M8N. 
  -> Download the U-center application (its free software. Yaay!) and change the update rate of the gps, the protocol to ubx-posllh as well it's baud rate to match the baud rate in this project.
  
  -> link to help for this : https://www.youtube.com/watch?v=TwhCX0c8Xe0&t=1639s
  
3) IMU : MPU9250. Use the slightly modified(i simply changed the scaling) library on my repo.

4) Microcontroller : An uno or mega will work but will take a whole lot of space. A nano will work but I like going old school so I went with a pro mini, 16Mhz, 5V version.

#SETTING UP :
You will have to create a setup similar to my setup (checkout "images" folder). I 3D-printed an open casing to hold the gps, mpu and the mcu in the same place
The casing reduces the chances of connections going lose on the fly.

In the setup():
Once done with fitting everything in the casing/harness, Find the offsets for the IMU. This part is not too difficult. A file to do this will be uploaded very soon. If you're smart enough, you can figure out how to get the offsets on your own

"driver module.ino":
Calculate the top speed of your car. ESC's generally have a linear input/output curve unless tuned otherwise. Divide 500(assuming throttle/brake is 0 at pulse width 1500us) by the top speed to get the open loop gain for the speed control. Set the closed loop gain as half the open loop gain.
Find the maximum steering angle. Steering angle can also be assumed to vary linearly. Divide 500 by maximum steering angle to obtain open loop gain for steering. Leave the rest as is.


  
