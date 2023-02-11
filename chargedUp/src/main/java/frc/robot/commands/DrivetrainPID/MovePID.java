// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainPID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MovePID extends PIDCommand {
  /** Creates a new MovePID. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  public MovePID(DrivetrainSubsystem drive, Double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD),
        // This should return the measurement
        () -> drive.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> { drive.setRaw(output/180, DrivetrainConstants.kStopMotors); //Divides by 180 to make output in between -1 and 1
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drivetrainSubsystem = drive;
    addRequirements(m_drivetrainSubsystem);

    getController().setTolerance(DrivetrainConstants.kMoveTolerance);
  }

  @Override
  public void initialize() {
    m_drivetrainSubsystem.resetPosition();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atSetpoint() == true);
  }
}
