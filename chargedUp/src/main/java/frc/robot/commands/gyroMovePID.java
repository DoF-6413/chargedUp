// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class gyroMovePID extends PIDCommand {
  /** Creates a new gyroMovePID. */
  public gyroMovePID(GyroSubsystem gyro, DrivetrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD),
        // This should return the measurement
        () -> gyro.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> { drive.setRaw(output, 0);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, gyro);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
