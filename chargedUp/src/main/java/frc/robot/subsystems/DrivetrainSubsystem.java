// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import frc.robot.SimulationDevices.SparkMaxWrapper;

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

private static GyroSubsystem gyro = new GyroSubsystem();
private static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
public static Field2d m_field2d = new Field2d();
  public DifferentialDrivetrainSim m_drivetrainSimulator;

  private static EncoderSim m_leftSimEncoder; 
  private static EncoderSim m_rightSimEncoder; 


  public DrivetrainSubsystem() {

    leftLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[3], MotorType.kBrushless);
    rightLead = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[0], MotorType.kBrushless);

    leftFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[4], MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[1], MotorType.kBrushless);

    leftFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[5], MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(DrivetrainConstants.kDrivetrainCANIDs[2], MotorType.kBrushless);

    Arrays.asList(leftLead, leftFollower1, leftFollower2, rightLead, rightFollower1, rightFollower2)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

    encoderLeftLead = leftLead.getEncoder();
    encoderRightLead = rightLead.getEncoder();
    
    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    // todo: uncomment for conversion
    encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    // encoderLeftLead.setInverted(DrivetrainConstants.kLeftInverted);

    leftFollower1.follow(leftLead);
    leftFollower2.follow(leftLead);


    encoderRightLead = rightLead.getEncoder();

    // todo: uncomment for conversion
    // encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToFeat);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);

    rightFollower1.follow(rightLead);
    rightFollower2.follow(rightLead);
    
    
    // Encoder fakeLeftEncoder = new Encoder(0, 1);
    // Encoder fakeRightEncoder = new Encoder(2, 3);

    diffDrive = new DifferentialDrive(leftLead, rightLead);
    
    m_odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), DrivetrainSubsystem.getDistanceLeaftlead(), DrivetrainSubsystem.getDistanceRigthlead());

       
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
    return (encoderLeftLead != null) ? encoderLeftLead.getPositionConversionFactor() : 00;

  }
  public static double getDistanceRigthlead(){
    return  (encoderRightLead != null) ? encoderRightLead.getPositionConversionFactor() : 00;

  }


  public static void updateOdometry(){
    m_odometry.update(gyro.getRotation2d(), getDistanceRigthlead(), getDistanceLeaftlead());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
  
    SmartDashboard.putData("Field", m_field2d);
 }
  

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("Drivetrain Position", this.getPositionRightLead());
  }
  
    public double getSimDrawnCurrentAmps() {
      return m_drivetrainSimulator.getCurrentDrawAmps();
    }

    public CANSparkMax getLeftMotor(){
      return leftLead;
    }

    public CANSparkMax getrightMotor(){
      return rightLead;
    }

    public EncoderSim getRightSimEncoder(){
      return m_rightSimEncoder;
    }

    public EncoderSim getLeftSimEncoder(){
      return m_leftSimEncoder;
    }
}
