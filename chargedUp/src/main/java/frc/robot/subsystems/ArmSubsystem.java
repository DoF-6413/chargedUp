// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
private final CANSparkMax m_leftRotationMotor;
private final CANSparkMax m_rightRotationMotor;
private final RelativeEncoder m_RotationEncoder;


private final TalonFX m_endEffectorMotor;
// private final RelativeEncoder m_endEffectorEncoder;

private final TalonFX m_telescopingMotor;
// private final RelativeEncoder m_telescopingEncoder;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftRotationMotor = new CANSparkMax(8, MotorType.kBrushless);
    m_rightRotationMotor = new CANSparkMax(9, MotorType.kBrushless);
    m_rightRotationMotor.follow(m_leftRotationMotor);
    m_RotationEncoder = m_leftRotationMotor.getEncoder();
    
    // m_RotationEncoder = m_leftRotationMotor.getEncoder();
    m_rightRotationMotor.follow(m_leftRotationMotor);
    
    m_endEffectorMotor = new TalonFX(16);
    m_endEffectorMotor.setNeutralMode(NeutralMode.Brake);
    
    m_telescopingMotor = new TalonFX(15);

    // DigitalInput toplimitSwitch = new DigitalInput(ArmConstants.kLimitSwitches[0]);
    // DigitalInput bottomlimitSwitch = new DigitalInput(ArmConstants.kLimitSwitches[1]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("RotationPosition",m_RotationEncoder.getPosition());
    SmartDashboard.putNumber("EndEffectorPosition", m_endEffectorMotor.getSelectedSensorPosition());
    
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
}
