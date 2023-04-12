// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescoperConstants;

public class TelescoperPIDSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new TelescoperPIDSubsystem. */
  
private final TalonFX m_telescopingMotor;
private final SimpleMotorFeedforward m_feedForward;

  public TelescoperPIDSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            TelescoperConstants.kTelescoperP,
            TelescoperConstants.kTelescoperI,
            TelescoperConstants.kTelescoperD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              TelescoperConstants.kTelescoperMaxVelocity, 
              TelescoperConstants.kTelescoperMaxAcceleration)));

              m_telescopingMotor = new TalonFX(TelescoperConstants.kTelescoperCANID);
              m_telescopingMotor.setNeutralMode(NeutralMode.Brake);
              m_telescopingMotor.setInverted(TelescoperConstants.kIsTelescoperInverted);
              StatorCurrentLimitConfiguration m_currentLimitConfig = new StatorCurrentLimitConfiguration(
                    TelescoperConstants.kIsTelescoperCurrentLimitEnabled, //Is enabled?
                    TelescoperConstants.kTelescoperContinuousCurrent, //Continuous Current Limit
                    TelescoperConstants.kTelescoperPeakCurrent, //Peak Current Limit
                    TelescoperConstants.kTelescoperMaxTimeAtPeak); //Time Allowed to be at Peak Current Limit
                    m_telescopingMotor.configStatorCurrentLimit(m_currentLimitConfig);
                    //Position Conversion Factor
              m_telescopingMotor.configSelectedFeedbackCoefficient(TelescoperConstants.kTelescopePositionConversionFactor);
        
              m_feedForward = new SimpleMotorFeedforward (
                TelescoperConstants.ksVolts,
                TelescoperConstants.kvVoltSecondPerMeter,
                TelescoperConstants.kaVoltsSecondsSquaredPerMeter
              );
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_telescopingMotor.set(ControlMode.Current, output);
    
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_telescopingMotor.getSelectedSensorPosition();
  }

  public void resetTelescoperPosition(){
    m_telescopingMotor.setSelectedSensorPosition(0);
  }

  public void telescoperCurrentLimit(double continuousCurrent, double maxCurrent){
    StatorCurrentLimitConfiguration m_currentLimitConfig = new StatorCurrentLimitConfiguration(
      true, //Is enabled?
      continuousCurrent, //Continuous Current Limit
      maxCurrent, //Peak Current Limit
      5.0); //Time Allowed to be at Peak Current Limit

      m_telescopingMotor.configStatorCurrentLimit(m_currentLimitConfig);
}

public double telecoperCurrent(){
return m_telescopingMotor.getStatorCurrent();
}
}
