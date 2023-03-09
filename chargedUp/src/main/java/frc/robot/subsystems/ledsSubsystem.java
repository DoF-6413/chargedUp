// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PWM;



public class ledsSubsystem extends SubsystemBase {
  private final PWM pwm;
  /** Creates a new ledsSubsystem. */
  public ledsSubsystem() {
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
  public void LEDPrimerPatron(){
    pwm.setSpeed(0.61);//red
  }
  public void LEDNewishPath(){
    pwm.setSpeed(0.87);//blue
  }
  public void LEDGetOntoChargingStation(){
    pwm.setSpeed(0.73);//green
  }
  public void LEDPatronNormal(){
    pwm.setSpeed(0.93);//white
  }

  @Override
  public void periodic() {

    }


    // This method will be called once per scheduler run
  }

