// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmMotor;

public class ArmPIDSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmPIDSubsystem. */
  private final CANSparkMax m_leftRotationMotor;
  private final CANSparkMax m_rightRotationMotor;
  private final RelativeEncoder m_RotationEncoder;
  private final AnalogPotentiometer m_pot;
  private final ArmFeedforward m_armFeedForward;

  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new ProfiledPIDController(
            // The PID gains
            ArmConstants.kRotationP,
            ArmConstants.kRotationI,
            ArmConstants.kRotationD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                ArmConstants.kArmMaxVelocity,
                ArmConstants.kArmMaxAcceleration)));

    m_leftRotationMotor = new CANSparkMax(ArmMotor.leftRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor = new CANSparkMax(ArmMotor.rightRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor.follow(m_leftRotationMotor);
    m_leftRotationMotor.setIdleMode(IdleMode.kBrake);
    m_rightRotationMotor.setIdleMode(IdleMode.kBrake);
    m_leftRotationMotor.setSmartCurrentLimit(60);
    m_rightRotationMotor.setSmartCurrentLimit(60);
    m_RotationEncoder = m_leftRotationMotor.getEncoder();
    m_RotationEncoder.setPositionConversionFactor(ArmConstants.kRotationPositionConversion);
    m_pot = new AnalogPotentiometer(
      ArmConstants.kpotetiometerPort, 
      ArmConstants.kpotetiometerRange,
      ArmConstants.kpotentiometerOffset
      );

    m_armFeedForward = new ArmFeedforward(
      ArmConstants.ksVolts, 
      ArmConstants.kgVolts, 
      ArmConstants.kvVoltSecondPerMeter,
      ArmConstants.kaVoltsSecondsSquaredPerMeter
    );

    m_controller.setTolerance(ArmConstants.kRotationTolerance);
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_RotationEncoder.getPosition() + ArmConstants.kArmOffsetRads;
  }

  /** RADIANS NOT DEGREES :) */
  @Override
  protected void useOutput(double output, State setpoint) {
    // TODO Auto-generated method stub
    double feedforward = m_armFeedForward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_leftRotationMotor.setVoltage(output + feedforward);
  }

  public void updateGoal(double increment){
    setGoal(m_controller.getGoal().position + increment);
  }

  public void setVoltage(double voltage){
    m_leftRotationMotor.setVoltage(voltage);
  }
  
  public void resetRotationPosition(double resetValue){
    m_RotationEncoder.setPosition(resetValue);
  }

  public double getPotentiometer(){
    
    return m_pot.get();
  }

  public boolean atGoal(){
    return m_controller.atGoal();
  }

  public double leftCurrent(){
    return m_leftRotationMotor.getOutputCurrent();
  }

  public double rightCurrent(){
    return m_rightRotationMotor.getOutputCurrent();
  }

  public void updateAcceleration(double accel){
    m_controller.setConstraints(new TrapezoidProfile.Constraints(ArmConstants.kArmMaxVelocity, accel));
  }

  public void updatePercentOutput(double percent){
    m_leftRotationMotor.set(percent);
  }



}
