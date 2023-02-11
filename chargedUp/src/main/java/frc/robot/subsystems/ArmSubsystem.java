// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
private final TalonFX m_leftRotationMotor;
private final TalonFX m_rightRotationMotor;
// private final RelativeEncoder m_RotationEncoder;

private final TalonFX m_endEffectorMotor;
// private final RelativeEncoder m_endEffectorEncoder;

private final TalonFX m_telescopingMotor;
// private final RelativeEncoder m_telescopingEncoder;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftRotationMotor = new TalonFX(ArmConstants.armCANIDs[0]);
    m_rightRotationMotor = new TalonFX(ArmConstants.armCANIDs[1]);
    
    
    // m_RotationEncoder = m_leftRotationMotor.getEncoder();
    m_rightRotationMotor.follow(m_leftRotationMotor);
    
    m_endEffectorMotor = new TalonFX(ArmConstants.armCANIDs[2]);
    m_endEffectorMotor.setNeutralMode(NeutralMode.Brake);
    
    m_telescopingMotor = new TalonFX(ArmConstants.armCANIDs[3]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("RotationPosition", m_leftRotationMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("EndEffectorPosition", m_endEffectorMotor.getSelectedSensorPosition());
    
  }

  public double getRotationPosition(){
    return m_leftRotationMotor.getSelectedSensorPosition();
  }

  public void spinRotationMotors(double speed){
    m_leftRotationMotor.set(TalonFXControlMode.PercentOutput,speed);
    SmartDashboard.putNumber("speed", speed);
  }

  public void spinEndEffector(double speed){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, speed);
  }



  public void stopEndEffector(){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void resetRotationPosition(){
    m_leftRotationMotor.setSelectedSensorPosition(0);
    m_rightRotationMotor.setSelectedSensorPosition(0);
  }
}
