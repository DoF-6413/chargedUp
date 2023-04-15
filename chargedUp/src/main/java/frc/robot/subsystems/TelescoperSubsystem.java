// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescoperConstants;

public class TelescoperSubsystem extends SubsystemBase {
  /** Creates a new TelescoperSubsystem. */
  
private final TalonFX m_telescopingMotor;
  public TelescoperSubsystem() {
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboardCalls();
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("TelescoperPosition", getTelescoperPosition());
    
    // SmartDashboard.putNumber("TelescoperCurrent", telecoperCurrent());
    
   
    
  }

  
  public double getTelescoperPosition(){
    return m_telescopingMotor.getSelectedSensorPosition();
  }

  public void resetTelescoperPosition(){
    m_telescopingMotor.setSelectedSensorPosition(0);
  }

  public void spinTelescopingMotor(double speed){
    m_telescopingMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopTelescopingMotor(){
    m_telescopingMotor.set(TalonFXControlMode.PercentOutput, 0);
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
