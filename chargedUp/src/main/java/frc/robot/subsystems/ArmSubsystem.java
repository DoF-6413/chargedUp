// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armCANIDs[0], MotorType.kBrushless);

    armEncoder = armMotor.getEncoder();

    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public void resetPosition(){
    armEncoder.setPosition(0);
  }

  public void spinMotor(double percentOutput){
    System.out.println("is spinning");
    armMotor.set(percentOutput); // 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmPosition", this.getPosition());
  }
}
