// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmMotor;

public class ArmSubsystem extends SubsystemBase {
private final CANSparkMax m_leftRotationMotor;
private final CANSparkMax m_rightRotationMotor;
private final RelativeEncoder m_RotationEncoder;
private final AnalogPotentiometer m_pot;
private final ArmFeedforward m_armFeedForward;
private double m_feedForwardValue;
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
    m_pot = new AnalogPotentiometer(ArmConstants.kpotetiometerPort, ArmConstants.kpotetiometerRange, ArmConstants.kpotentiometerOffset);

    m_armFeedForward = new ArmFeedforward(ArmConstants.ksVolts, ArmConstants.kgVolts, ArmConstants.kvVoltSecondPerMeter, ArmConstants.kaVoltsSecondsSquaredPerMeter);
    m_feedForwardValue = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboardCalls();
  }

  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("RotationPosition", getRotationPosition());  
    SmartDashboard.putNumber("Arm Current", m_leftRotationMotor.getOutputCurrent()) ; 
    SmartDashboard.putNumber("Potentiometer Reading", m_pot.get());
  }

  public double getRotationPosition(){
    return m_RotationEncoder.getPosition();
  }

  public double getRotationVelocity(){
    return m_RotationEncoder.getVelocity();
  }
  
  public void resetRotationPosition(){
    m_RotationEncoder.setPosition(0);
  }

  public void spinRotationMotors(double speed){
    m_leftRotationMotor.set(speed);
    SmartDashboard.putNumber("Rotation speed", speed);
  }

  public void updateFeedForward(double Position, double Velocity){
    m_feedForwardValue =  m_armFeedForward.calculate(Position, Velocity);
  }
  public void rotationVoltage(double output){
    //maybe set velocity as 0
    // updateFeedForward(Units.degreesToRadians(getRotationPosition() - 95), 0);
    m_leftRotationMotor.setVoltage(output
    //  + m_feedForwardValue
     );
  }

  public void stopRotationMotors(){
    m_leftRotationMotor.set(0);
  }

  public double getPotentiometer(){
    return m_pot.get();
  }

  public Boolean isInFramePerimeter(){
    return ((this.getRotationPosition() < 35) && (this.getRotationPosition() > -35 )) ?  true :  false;
  }

}
