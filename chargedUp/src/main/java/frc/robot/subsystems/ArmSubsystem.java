// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmMotor;

public class ArmSubsystem extends SubsystemBase {
private final CANSparkMax m_leftRotationMotor;
private final CANSparkMax m_rightRotationMotor;
private final RelativeEncoder m_RotationEncoder;


private final TalonFX m_endEffectorMotor;

private final TalonFX m_telescopingMotor;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftRotationMotor = new CANSparkMax(ArmMotor.leftRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor = new CANSparkMax(ArmMotor.rightRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor.follow(m_leftRotationMotor);
    m_leftRotationMotor.setIdleMode(IdleMode.kBrake);
    m_rightRotationMotor.setIdleMode(IdleMode.kBrake);
    m_leftRotationMotor.setSmartCurrentLimit(ArmConstants.kRotationCurrentLimit);
    
    m_RotationEncoder = m_leftRotationMotor.getEncoder();
    m_RotationEncoder.setPositionConversionFactor(ArmConstants.kRotationPositionConversion);
    
    m_rightRotationMotor.follow(m_leftRotationMotor);
    
    m_endEffectorMotor = new TalonFX(ArmMotor.endEffectorMotor.CAN_ID);
    m_endEffectorMotor.setNeutralMode(NeutralMode.Brake);
    
    m_telescopingMotor = new TalonFX(ArmMotor.telescopingMotor.CAN_ID);
    m_telescopingMotor.setNeutralMode(NeutralMode.Brake);
    m_telescopingMotor.setInverted(ArmConstants.kIsTelescoperInverted);

    StatorCurrentLimitConfiguration m_currentLimitConfig = new StatorCurrentLimitConfiguration(
          ArmConstants.kIsTelescoperCurrentLimitEnabled, //Is enabled?
          ArmConstants.kTelescoperContinuousCurrent, //Continuous Current Limit
          ArmConstants.kTelescoperPeakCurrent, //Peak Current Limit
          ArmConstants.kTelescoperMaxTimeAtPeak); //Time Allowed to be at Peak Current Limit
          m_telescopingMotor.configStatorCurrentLimit(m_currentLimitConfig);
          //Position Conversion Factor
    m_telescopingMotor.configSelectedFeedbackCoefficient(ArmConstants.kTelescopePositionConversionFactor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboardCalls();
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("RotationPosition", m_RotationEncoder.getPosition());
    SmartDashboard.putNumber("EndEffectorPosition", m_endEffectorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("TelescoperPosition", m_telescopingMotor.getSelectedSensorPosition());

    
  }

  public double getRotationPosition(){
    return m_RotationEncoder.getPosition();
  }
  
  public void resetRotationPosition(){
    m_RotationEncoder.setPosition(0);
  }

  public void spinRotationMotors(double speed){
    m_leftRotationMotor.set(speed);
    SmartDashboard.putNumber("Rotation speed", speed);
  }

  public void stopRotationMotors(){
    m_leftRotationMotor.set(0);
  }


  public double getEndEffectorPosition(){
    return m_endEffectorMotor.getSelectedSensorPosition();
  }

  public void resetEndEffectorPosition(){ 
    m_endEffectorMotor.setSelectedSensorPosition(0);
  }

  public void spinEndEffector(double speed){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, speed);
  }


  public void stopEndEffector(){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, 0);
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
