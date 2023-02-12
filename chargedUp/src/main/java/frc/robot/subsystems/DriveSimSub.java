// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSimSub extends SubsystemBase {
  /** Creates a new SimulationSubsystem. */
  
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private double appliedVoltsLeft = 0.0;
  private double appliedVoltsRight = 0.0;
  private boolean closedLoop = false;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private static AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  // private static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
  // private Field2d m_field = new Field2d();

  private static Encoder encoderLeftLead = new Encoder(4, 5);
  private static Encoder encoderRightLead = new Encoder(2, 3);

  private static EncoderSim m_rightSimEncoder;
  private static EncoderSim m_leftSimEncoder;
  
  private PIDController leftPID =
  new PIDController(0.0, 0.0, 0.0, DrivetrainConstants.loopPeriodSecs);
private PIDController rightPID =
  new PIDController(0.0, 0.0, 0.0, DrivetrainConstants.loopPeriodSecs);

  public DriveSimSub() {
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            // todo: will fix :)
              DCMotor.getNEO(3),
              DrivetrainConstants.kgearing,
              DrivetrainConstants.kMOI,
              DrivetrainConstants.kMass,
              DrivetrainConstants.kwheelRadiusMeters,
              DrivetrainConstants.ktrackWidth,
              // The standard deviations for measurement noise:
              // x and y:          0.001 m
              // heading:          0.001 rad
              // l and r velocity: 0.1   m/s
              // l and r position: 0.005 m
              VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

              

              m_leftSimEncoder = new EncoderSim(encoderLeftLead);
              m_rightSimEncoder = new EncoderSim(encoderRightLead);
  }

  public void updateInputs() {
    //   m_drivetrainSimulator.setInputs(
    //     (-leftVolts * RobotController.getBatteryVoltage()),
    //       rightVolts* RobotController.getBatteryVoltage());
      
      // double drawCurrent = m_drivetrainSimulator.getCurrentDrawAmps();
      // double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
      // RoboRioSim.setVInVoltage(loadedVoltage);

      /*
       * Under here is the crazy
       */
    if (closedLoop) {
      double leftVolts = leftPID
          .calculate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters)
          + leftFFVolts;
      double rightVolts = rightPID
          .calculate(m_drivetrainSimulator.getRightVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters)

          + rightFFVolts;
      appliedVoltsLeft = leftVolts;
      appliedVoltsRight = rightVolts;
      m_drivetrainSimulator.setInputs(leftVolts, rightVolts);
      }

     
      m_leftSimEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
      m_leftSimEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    
      m_rightSimEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
      m_rightSimEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
      
      
    m_drivetrainSimulator.update(DrivetrainConstants.loopPeriodSecs);

  

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    // double leftPositionRad = m_drivetrainSimulator.getLeftPositionMeters() / DrivetrainConstants.kwheelRadiusMeters;
    double leftVelocityRadPerSec =
      m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters;
    // double[] leftCurrentAmps = new double[] {m_drivetrainSimulator.getLeftCurrentDrawAmps()};
    // double[] leftTempCelcius = new double[] {};
    
    // double rightPositionRad = m_drivetrainSimulator.getRightPositionMeters() / DrivetrainConstants.kwheelRadiusMeters;
    double rightVelocityRadPerSec =
      m_drivetrainSimulator.getRightVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters;
    // double[] rightCurrentAmps = new double[] {m_drivetrainSimulator.getRightCurrentDrawAmps()};
    // double[] rightTempCelcius = new double[] {};
    
    // double gyroConnected = true;
    // double lastGyroPosition = inputs.gyroYawPositionRad;
    // double gyroYawPositionRad =DifferentialDrivetrainSim.getHeading().getRadians() * -1;
    // double gyroYawVelocityRadPerSec =
    //     (inputs.gyroYawPositionRad - lastGyroPosition)
    //         / Constants.loopPeriodSecs;
  }
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    appliedVoltsLeft = leftVolts;
    appliedVoltsRight = rightVolts;
    m_drivetrainSimulator.setInputs(leftVolts, rightVolts);
    updateInputs();
    // m_drivetrainSimulator.update(DrivetrainConstants.loopPeriodSecs);
  }
  
  public void setVelocity(double leftVelocityRadPerSec,
  double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    updateInputs();
  }

  public void updateDashboard()
{
  SmartDashboard.putString("Running Sim", "Running Sim");
  SmartDashboard.putNumber("LeftVolts",  appliedVoltsLeft);
  SmartDashboard.putNumber("RightVolts",  appliedVoltsRight);
  SmartDashboard.putNumber("LeftVelocityMeters",  m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
  SmartDashboard.putNumber("LeftPositionMeters",  m_drivetrainSimulator.getLeftPositionMeters());
  SmartDashboard.putNumber("RightVelocityMeters",  m_drivetrainSimulator.getRightVelocityMetersPerSecond());
  SmartDashboard.putNumber("RighhtPositionMeters",  m_drivetrainSimulator.getRightPositionMeters());
  SmartDashboard.putNumber("RightSimEncoderDistance",  m_rightSimEncoder.getDistance());
  SmartDashboard.putNumber("LeftSimEncoderDistance",  m_leftSimEncoder.getDistance());
}
  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);
    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }

// //   // @Override
//   public void field(){
//   // public void periodic(){
//     m_odometry.update(m_gyro.getRotation2d(),
//     m_drivetrainSimulator.getLeftPositionMeters(),
//     m_drivetrainSimulator.getRightPositionMeters());
// m_field.setRobotPose(m_odometry.getPoseMeters());
// SmartDashboard.putData("Field", m_field);
//   }
}
