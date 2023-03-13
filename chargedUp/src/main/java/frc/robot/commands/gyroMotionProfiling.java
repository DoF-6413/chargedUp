// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class gyroMotionProfiling extends ProfiledPIDCommand {
  /** Creates a new gyroMotionProfiling. */
  public gyroMotionProfiling(GyroSubsystem gyro, DrivetrainSubsystem drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.35, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 0.2)),
        // This should return the measurement
        () -> gyro.getRoll(),
        // This should return the goal (can also be a constant)
        () -> 0,
        // This uses the output
        (output, setpoint) -> {drive.setRaw(-output, 0);
          // Use the output (and setpoint, if desired) here
          
        });
    // Use addRequirements() here to declare subsystem dependencies.
     // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(drive, gyro);
     // Configure additional PID options by calling `getController` here.
     getController().setTolerance(1);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
