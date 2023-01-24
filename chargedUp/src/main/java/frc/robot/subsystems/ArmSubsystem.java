// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armCANIDs[0], MotorType.kBrushless);
  private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
  private final ArmFeedforward m_armFeedForward = new ArmFeedforward(ArmConstants.kArmS, ArmConstants.kArmG, ArmConstants.kArmV);

  public ArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConstants.kArmP,
            ArmConstants.kArmI,
            ArmConstants.kArmD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConstants.kArmMaxVelocity, 
              ArmConstants.kArmMaxAcceleration)));

        // m_armEncoder.setPositionConversionFactor(ArmConstants.kArmPositionConversion);
        // setGoal(ArmConstants.kOffsetInitialPosition);
        System.out.println("Running");
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedForward = m_armFeedForward.calculate(setpoint.position, setpoint.velocity);
    System.out.println("Use Output Running");
    m_armMotor.set(feedForward + output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    
    return m_armEncoder.getPosition() + ArmConstants.kOffsetInitialPosition;
  }

  public void resetPosition(){
    System.out.println("Reset Position");
    m_armEncoder.setPosition(0);
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putNumber("Arm Position", getMeasurement());
  }
}
