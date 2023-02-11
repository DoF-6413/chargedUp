// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower1;
  private static CANSparkMax rightFollower1;
  private static CANSparkMax leftFollower2;
  private static CANSparkMax rightFollower2;


  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;

  private static DifferentialDrive diffDrive;

private static GyroSubsystem gyro;
private static DifferentialDriveOdometry m_odometry;
public final static Field2d m_field2d = new Field2d();


  public DrivetrainSubsystem() {

    leftLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);
    rightLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);

    leftFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[4], MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);

    leftFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[5], MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);

    leftLead.setIdleMode(IdleMode.kBrake);
    rightLead.setIdleMode(IdleMode.kBrake);

    leftFollower1.setIdleMode(IdleMode.kBrake);
    rightFollower1.setIdleMode(IdleMode.kBrake);

    leftFollower2.setIdleMode(IdleMode.kBrake);
    rightFollower2.setIdleMode(IdleMode.kBrake);



    encoderLeftLead = leftLead.getEncoder();
    encoderRightLead = rightLead.getEncoder();
    
    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    // todo: uncomment for conversion
    encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    // encoderLeftLead.setInverted(DrivetrainConstants.kLeftInverted);

    leftFollower1.follow(leftLead);
    leftFollower2.follow(leftLead);


    // encoderRightLead = rightLead.getEncoder();

    // todo: uncomment for conversion
    // encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);

    rightFollower1.follow(rightLead);
    rightFollower2.follow(rightLead);
    
     
    diffDrive = new DifferentialDrive(leftLead, rightLead);
    
    m_odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), DrivetrainSubsystem.getDistanceLeaftlead(), DrivetrainSubsystem.getDistanceRigthlead());

  
        SmartDashboard.putData("Field", m_field2d);
  }


   
  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public double getPositionLeftLead(){
    return encoderLeftLead.getPosition();
  }

  public double getPositionRightLead(){
    return encoderRightLead.getPosition();
  }

  public void resetPosition(){
    encoderLeftLead.setPosition(0);
  }
// use this later to set to specific pose by passing in an argument to this void
//   public void resetPose(){
// m_odometry.resetPosition(null, getDistance(), getDistance(), null);
//   }
  
  public static double getDistanceLeaftlead(){
    return encoderLeftLead.getPositionConversionFactor();

  }
  public static double getDistanceRigthlead(){
    return encoderRightLead.getPositionConversionFactor();

  }


  public static void updateOdometry(){
    m_odometry.update(gyro.getRotation2d(), getDistanceRigthlead(), getDistanceLeaftlead());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
 }
  

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("Drivetrain Position", this.getPositionRightLead());
  }
  
}
