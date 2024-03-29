// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public final PWM pwm;
  public LEDSubsystem() {
    pwm = new PWM(1);
    
  }

  public void setLeds(double somevalue){
    pwm.setSpeed(somevalue);
  }

  
  public void NeedACone(){
    pwm.setSpeed(0.69);//yellow 
    
  }

  public void NeedACube(){
    pwm.setSpeed(0.91);//violet

  }
  public void SetLedsOff(){
    pwm.setSpeed(0.99);//off leds
  }

  public void LEDBlue(){
    pwm.setSpeed(0.87);//blue
  }
  public void LEDWhite(){
    pwm.setSpeed(0.93);//white
     }
  
  public void LEDTimer(){
    if ( Timer.getMatchTime() > 30 ){
      pwm.setSpeed(0.77);//solid green
      }
        else if ((Timer.getMatchTime() < 30) && ( Timer.getMatchTime() > 15)){
   
        pwm.setSpeed(0.61);  //solid red
        }
        
          else if (Timer.getMatchTime() < 15){
           pwm.setSpeed(-0.1);//blink in red
          }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
