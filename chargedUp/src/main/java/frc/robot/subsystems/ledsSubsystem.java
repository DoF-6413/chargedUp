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


  public void setLedsOff(){
    System.out.println("setLedsoff runnning");
    pwm.setSpeed(00);    
  }

  @Override
  public void periodic() {

    }
   
    // This method will be called once per scheduler run
  }

