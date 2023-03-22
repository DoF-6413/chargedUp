// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotationPID extends ProfiledPIDCommand {
  /** Creates a new RotationPID. */
  ArmSubsystem m_armSubsystem;

  public RotationPID(ArmSubsystem arm, double targetPosition) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            ArmConstants.kRotationP,
            ArmConstants.kRotationI,
            ArmConstants.kRotationD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConstants.kArmMaxVelocity, ArmConstants.kArmMaxAcceleration)),
        // This should return the measurement
        () -> arm.getRotationPosition(),
        // This should return the goal (can also be a constant)
        () -> targetPosition,
        // This uses the output
        (output, setpoint) -> {
          arm.spinRotationMotors(output);
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
    addRequirements(m_armSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController()
        .setTolerance(ArmConstants.kRotationTolerance);
       
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
