// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenixpro.signals.System_StateValue;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ledsSubsystem extends SubsystemBase {
  public final PWM pwm;
  /** Creates a new ledsSubsystem. */
  public ledsSubsystem() {
    pwm = new PWM(1);
  }

  public void setLeds(double somevalue){
    pwm.setSpeed(somevalue);  
    System.out.println ("CAMBIA DE COLOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
  }

  
  public void NeedACone(){

    //yellow 
    System.out.println("yellow!!!!!!!!!!!!");
  }

  public void NeedACube(){
    pwm.setSpeed(0.91);//violet
    System.out.println("violet!!!!!!!!!!!!!!!!!!!!!!1");
  }
  public void SetLedsOff(){
    pwm.setSpeed(0.99);//off leds
    System.out.println("se fue la luz!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  public void LEDPrimerPatron(){
    pwm.setSpeed(0.61);//red
    System.out.println("el primer patron");
  }
  public void LEDNewishPath(){
    pwm.setSpeed(0.87);//blue
    System.out.println("newishpath vamooooooooo");
  }
  public void LEDGetOntoChargingStation(){
    pwm.setSpeed(0.73);//green
    System.out.println("vamo pa la charge estation!!!!!");
  }
  public void LEDPatronNormal(){
    pwm.setSpeed(0.93);//white
    System.out.println("patron normal!!!!!!!!!!!");
  }

  @Override
  public void periodic() {

    }


    // This method will be called once per scheduler run
  }

