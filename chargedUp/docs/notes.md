# Programming Notes
![](./qrcode_github.com.png)

# 1-21-23

Updates from January 21st 2023




## Andy

- Today I commit the code of the buttons in the smart dashboard to I make with Irae to we made 1/19/2023
- I create the folder for the notes of the day 
- I help Nancy to calibrate the camera in photon vision 

## Nancy
- I set up the new camera onto minnie. I changed the name of the previous camera and named the new one the same as the previous one. This way I was still able to run the code that was on minnie using the new camera. Then I **calibrated the new camera to read the apriltags in 3D** and tested that minnie was still able to move and do the autonomous command that I told it to do in the code using the new camera. 


- **The new arducam is named "logi...."**

## Liz

Today I looked into **robot simulation code**. Robot simulation would enable us to test the robot without having to use an actual robot. Then I shifted gears towards **PID Tuning for an arm mechanism.** The PID Tuning would spin the arm to specific positions. However, I need to ramp the speed of the arm, as well as tune PID.

# 1-22-23

## Mentors

- Today we discussed trapezoidal motion smoothingas a possible solution to help the arm move smoother [Click on this Link to WPIlib docs for Trapezoidal smoothing](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html)

## Nancy

- made the driveToTag command run in autonomous 
- made a vision constants inside constants   that holds a list of different setpoints
- added the get best fiducial to the initialize in the runToTag and made an array so that it runs a different distance based on the fiducial tag that the camera sees
- We initially had the get best fiducial in the robot container but since it would be constantly changing it did not accept it so moved it to the command

# 1-23-23

## Nancy

- Added a sensor subsystem 
- Imported the ultrasonic and smartdashboard imports in the sensor subsystem

### Issues for this week: 
- Add ultrasonic sensor to subsystem and create code that will print distance onto the smartdashboard
