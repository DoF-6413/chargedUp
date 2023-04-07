// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DriveMotor;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.SimulationDevices.SparkMaxWrapper;
import frc.robot.subsystems.PoseEstimator;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private static CANSparkMax leftLead;
  private static CANSparkMax rightLead;
  private static CANSparkMax leftFollower1;
  private static CANSparkMax rightFollower1;
  
  private static RelativeEncoder encoderLeftLead;
  private static RelativeEncoder encoderRightLead;
  
  private static DifferentialDrive diffDrive;
  public static DifferentialDriveKinematics m_Kinematics;
  public static DifferentialDriveWheelSpeeds m_WheelSpeeds;
  
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  
  private static Encoder simEncoderRightRep;
  private static Encoder simEncoderLeftRep;
  
  private static EncoderSim m_rightSimEncoder;
  private static EncoderSim m_leftSimEncoder;
  
  public static Field2d m_field2d;
  
  private static GyroSubsystem gyro;

  SimDouble gyroAngleSim;
  
  public DrivetrainSubsystem(GyroSubsystem m_gyro) {
    
    leftLead = new SparkMaxWrapper(DriveMotor.leftLead.CAN_ID, MotorType.kBrushless);
    rightLead = new SparkMaxWrapper(DriveMotor.rightLead.CAN_ID, MotorType.kBrushless);

    leftLead.setInverted(DrivetrainConstants.kLeftInverted);
    rightLead.setInverted(DrivetrainConstants.kRightInverted);

    leftFollower1 = new SparkMaxWrapper(DriveMotor.leftFollower1.CAN_ID, MotorType.kBrushless);
    rightFollower1 = new SparkMaxWrapper(DriveMotor.rightFollower1.CAN_ID, MotorType.kBrushless);

    Arrays.asList(leftLead, leftFollower1, rightLead, rightFollower1)
        .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

    leftFollower1.follow(leftLead);
    rightFollower1.follow(rightLead);

    encoderLeftLead = leftLead.getEncoder();
    encoderRightLead = rightLead.getEncoder();

    encoderLeftLead.setPositionConversionFactor(DrivetrainConstants.kTicksToMeters);
    encoderRightLead.setPositionConversionFactor(DrivetrainConstants.kTicksToMeters);

    gyro = m_gyro;
    diffDrive = new DifferentialDrive(leftLead, rightLead);
    
    // Todo: Find out what this does
    diffDrive.setSafetyEnabled(false);
    
    simEncoderLeftRep = new Encoder(4, 5);
    simEncoderRightRep = new Encoder(2, 3);
    
    simEncoderLeftRep.setDistancePerPulse(0.0238095238);
    simEncoderRightRep.setDistancePerPulse(0.0238095238);
    
    simEncoderLeftRep.setReverseDirection(true);

    m_Kinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);

    if (RobotBase.isSimulation()) {
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
          // todo: change numMotors -> 2 and update the rest of the values for new robot
          DCMotor.getNEO(2),
          DrivetrainConstants.kgearing,
          3,
          DrivetrainConstants.kMass,
          DrivetrainConstants.kwheelRadiusMeters,
          DrivetrainConstants.kTrackWidth,
          // The standard deviations for measurement noise:
          // x and y: 0.001 m
          // heading: 0.001 rad
          // l and r velocity: 0.1 m/s
          // l and r position: 0.005 m
          VecBuilder.fill(5, 5, 0.001, 0.1, 0.1, 0.005, 0.005));

      m_leftSimEncoder = new EncoderSim(simEncoderLeftRep);
      m_rightSimEncoder = new EncoderSim(simEncoderRightRep);

      gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value + "]").getDouble("Angle");

      m_field2d = new Field2d();

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboardCalls();
  }
  
  public void SmartDashboardCalls() {
    // SmartDashboard.putNumber("Drivetrain Right", this.getPositionRightLead());
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putString("Pose", getPose().toString());
    // SmartDashboard.putNumber("Drivetrain Position", getPosition());
    // SmartDashboard.putNumber("Left Lead", leftLead.get());
    // SmartDashboard.putNumber("Right Lead", rightLead.get());
    // SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Left position", getPositionLeftLead());
    SmartDashboard.putNumber("Right position", getPositionRightLead());
  }

  public void setRaw(double driveValue, double turnValue) {
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public CANSparkMax getLeftMotor() {
    return leftLead;
  }

  public CANSparkMax getRightMotor() {
    return rightLead;
  }

  public double getPositionLeftLead() {
    if (RobotBase.isSimulation()) {
      return simEncoderLeftRep.getDistance();
    } else {
      return encoderLeftLead.getPosition();
    }
  }

  public double getPositionRightLead() {
    if (RobotBase.isSimulation()) {
      return simEncoderRightRep.getDistance();
    } else {
      return encoderRightLead.getPosition();
    }
  }

  public double getPosition() {
    return (getPositionLeftLead() + getPositionRightLead()) / 2;
  }

  
  public void resetPosition() {
    encoderLeftLead.setPosition(0);
    encoderRightLead.setPosition(0);
  }


  
  /*
  Note: In the example code, he manually sets one velocity outfut to be negative(inverted)? 
  He does that with pos as well, but we don't and havent had any problems so Im not gonna manually invert it 
  (he also did do te set inverted AND he has the same motors, encoders, and motor controllers as us)
  Could be a point of failure 
  */
  public double getVelocityLeftLead(){
    return encoderLeftLead.getVelocity();
  }

  public double getVelocityRightLead(){
    return encoderRightLead.getVelocity();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    if(RobotBase.isSimulation()){
    return new DifferentialDriveWheelSpeeds(simEncoderLeftRep.getRate(), simEncoderRightRep.getRate());
    }else{
      return new DifferentialDriveWheelSpeeds(getVelocityLeftLead(), getVelocityRightLead());
    }

  }

  public void tankDrive(double leftVolts, double rightVolts){
    leftLead.setVoltage(leftVolts);
    rightLead.setVoltage(rightVolts);
    diffDrive.feed();
  }

  /** */
  public void switchIdleMode(boolean setCoast){
if (setCoast == true){
  Arrays.asList(leftLead, leftFollower1, rightLead, rightFollower1)
        .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));
} else if (setCoast ==false){
  Arrays.asList(leftLead, leftFollower1, rightLead, rightFollower1)
        .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
}
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(
      (rightLead.get() * RobotController.getBatteryVoltage()),
      leftLead.get() * RobotController.getBatteryVoltage());
      
      m_drivetrainSimulator.update(0.020);
      
    m_leftSimEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftSimEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    m_rightSimEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightSimEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    
    gyroAngleSim.set(m_drivetrainSimulator.getHeading().getDegrees());
  }
  
}
