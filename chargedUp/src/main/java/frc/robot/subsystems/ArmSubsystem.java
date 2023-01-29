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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
private final CANSparkMax m_leftRotationMotor;
private final RelativeEncoder m_leftRotationEncoder;

private final CANSparkMax m_rightRotationMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftRotationMotor = new CANSparkMax(ArmConstants.armCANIDs[0], MotorType.kBrushless);
    m_rightRotationMotor = new CANSparkMax(ArmConstants.armCANIDs[1], MotorType.kBrushless);
    
    m_leftRotationEncoder = m_leftRotationMotor.getEncoder();

    m_rightRotationMotor.follow(m_leftRotationMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("armPosition",m_leftRotationEncoder.getPosition());
  }

  public double getRotationPosition(){
    return m_leftRotationEncoder.getPosition();
  }

  public void spinRotationMotors(double speed){
    m_leftRotationMotor.set(speed);
    m_rightRotationMotor.set(speed);
    SmartDashboard.putNumber("speed", speed);
  }

  public void resetRotationPosition(){
    m_leftRotationEncoder.setPosition(0);
  }
}
