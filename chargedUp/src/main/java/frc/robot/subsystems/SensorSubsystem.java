// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class SensorSubsystem extends SubsystemBase {
  /** Creates a new SensorSubsystem. */
  AnalogInput input = new AnalogInput(0);
  AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);
  

  
  //  AnalogPotentiometer pot = new AnalogPotentiometer(input, 100, 30);
  public SensorSubsystem() {
    input.setAverageBits(2);
 
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Drivetrain Position", input.get);
    // SmartDashboard.putString("Drivetrain Position 0", input.toString());
    // SmartDashboard.putNumber("Drivetrain Position 1", input.set);
    // SmartDashboard.putNumber("Drivetrain Position 2", input.get());
  }
}
