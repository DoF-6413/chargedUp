// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower;
  private static CANSparkMax rightFollower;

  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;

  private static DifferentialDrive diffDrive;
 




  public DrivetrainSubsystem() {
    leftLead = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);
    rightLead = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);
    // leftFollower = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);
    // rightFollower = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);

    encoderLeftLead = leftLead.getEncoder();
    // leftFollower.follow(leftLead);

    encoderRightLead = rightLead.getEncoder();
    // rightFollower.follow(rightLead);

    //invert one of the leads :)

    diffDrive = new DifferentialDrive(leftLead, rightLead);
  }

  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public double getPosition(){
    return encoderLeftLead.getPosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drivetrain Position", this.getPosition());
  }
}
