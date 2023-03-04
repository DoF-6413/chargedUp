// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.PWM;

public class ledsSubsystem extends SubsystemBase {
  private final PWM pwm;
  /** Creates a new ledsSubsystem. */
  public ledsSubsystem() {
    pwm = new PWM(1);
  }
  
  public void setmode(double mode){
    pwm.setSpeed(mode);  
  }

  public static enum BlinkLEDMode{
    SOLID_RED(0.61),
    SOLID_YELLOW(0.69),
    SOLID_LIME(0.73),
    SOLID_BLUE(0.87),
    SOLID_VIOLET(0.91),
    SOLID_WHITE(0.93),
    SOLID_BLACK(0.99);
    private final double value;

    BlinkLEDMode (double value){
    this.value=value;
    }
  }
  public void NeedACone(){
    
    setmode(BlinkLEDMode.SOLID_YELLOW.value); //yellow  
  }

  public void NeedACube(){
    setmode(BlinkLEDMode.SOLID_VIOLET.value);//violet
  }
  public void SetLedsOff(){
    setmode(BlinkLEDMode.SOLID_BLACK.value);    //off leds
  }
  public void LEDPrimerPatron(){
    setmode(BlinkLEDMode.SOLID_RED.value); //red
  }
  public void LEDNewishPath(){
    setmode(BlinkLEDMode.SOLID_BLUE.value);//blue
  }
  public void LEDGetOntoChargingStation(){
    setmode(BlinkLEDMode.SOLID_LIME.value);//green
  }
  public void LEDPatronNormal(){
    setmode(BlinkLEDMode.SOLID_WHITE.value);//white
  }

  @Override
  public void periodic() {

    }


    // This method will be called once per scheduler run
  }

