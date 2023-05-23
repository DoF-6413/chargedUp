// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static DifferentialDrive diffDrive;
  private final WPI_VictorSPX leftLead;
  private final WPI_VictorSPX rightLead; 
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightFollower;



  public DrivetrainSubsystem() {
    leftLead = new WPI_VictorSPX(DrivetrainConstants.kDrivetrainCANIDs[2]);
    rightLead = new WPI_VictorSPX(DrivetrainConstants.kDrivetrainCANIDs[0]);
    leftFollower = new WPI_VictorSPX(DrivetrainConstants.kDrivetrainCANIDs[1]);
    rightFollower = new WPI_VictorSPX(DrivetrainConstants.kDrivetrainCANIDs[3]);

    leftLead.setInverted(true);
    leftFollower.follow(leftLead);
    rightFollower.follow(rightLead);

    //invert one of the leads :)
    // leftLead.configSelectedFeedbackCoefficient(Units.inchesToMeters(6) /);

    diffDrive = new DifferentialDrive(rightLead,leftLead);
  }


   
  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getLeftPose(){
    return leftLead.getSelectedSensorPosition();
  }
  
}
