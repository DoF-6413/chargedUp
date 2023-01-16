// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.rmi.server.SocketSecurityException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower;
  private static CANSparkMax rightFollower;


  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;

  private static DifferentialDrive diffDrive;
 
  private static double m_output;

  private final SimpleMotorFeedforward m_drivetrainFeedforward;




  public DrivetrainSubsystem() {

    leftLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);
    rightLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);
    // leftFollower = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);
    // rightFollower = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);

    encoderLeftLead = leftLead.getEncoder();
    
    // todo: uncomment for conversion
    // encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    encoderLeftLead.setInverted(DrivetrainConstants.kLeftInverted);

     //Uncomment for follower
    // leftFollower.follow(leftLead);

    encoderRightLead = rightLead.getEncoder();

    // todo: uncomment for conversion
    // encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    encoderRightLead.setInverted(DrivetrainConstants.kRightInverted);

    //Uncomment for follower
    // rightFollower.follow(rightLead);
    
     
    diffDrive = new DifferentialDrive(leftLead, rightLead);

    m_drivetrainFeedforward = new SimpleMotorFeedforward(
          DrivetrainConstants.kS, DrivetrainConstants.kV);

    PIDController leftPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
    PIDController rightPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  }


   
  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public double getPosition(){
    return encoderLeftLead.getPosition();
  }

  public void resetPosition(){
    encoderLeftLead.setPosition(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
   
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("Drivetrain Position", this.getPosition());
  }

  // @Override
  // public void useOutput(double output, double setpoint) {
  //   SmartDashboard.putNumber("useOutput", output + m_drivetrainFeedforward.calculate(setpoint));
  //   SmartDashboard.putNumber("setpoint", setpoint);
  //   SmartDashboard.putNumber("output", output );
  //   // setRaw(output + m_drivetrainFeedforward.calculate(setpoint), 0.0);
  // }

  // @Override
  // public double getMeasurement() {
  //   // Return the process variable measurement here
  //   SmartDashboard.putNumber("measurment", encoderLeftLead.getVelocity());
  //   return 0;
  // }

  
}
