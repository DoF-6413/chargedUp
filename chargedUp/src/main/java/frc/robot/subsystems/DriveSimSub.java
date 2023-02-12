// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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

  }

  public void updateInputs() {
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

    m_drivetrainSimulator.update(DrivetrainConstants.loopPeriodSecs);
    double leftPositionRad = m_drivetrainSimulator.getLeftPositionMeters() / DrivetrainConstants.kwheelRadiusMeters;
    double leftVelocityRadPerSec =
      m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters;
    double leftAppliedVolts = appliedVoltsLeft;
    double[] leftCurrentAmps = new double[] {m_drivetrainSimulator.getLeftCurrentDrawAmps()};
    double[] leftTempCelcius = new double[] {};

    double rightPositionRad = m_drivetrainSimulator.getRightPositionMeters() / DrivetrainConstants.kwheelRadiusMeters;
    double rightVelocityRadPerSec =
      m_drivetrainSimulator.getRightVelocityMetersPerSecond() / DrivetrainConstants.kwheelRadiusMeters;
    double rightAppliedVolts = appliedVoltsRight;
    double[] rightCurrentAmps = new double[] {m_drivetrainSimulator.getRightCurrentDrawAmps()};
    double[] rightTempCelcius = new double[] {};

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
  }

  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }

  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);
    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }
}
