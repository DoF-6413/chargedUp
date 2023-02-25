// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionMovePID extends PIDCommand {
  /** Creates a new VisionMovePID. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;
  public VisionMovePID(DrivetrainSubsystem drive, VisionSubsystem vision) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> vision.distanceFinder().getX(),
        // This should return the setpoint (can also be a constant)
        () -> 9,
        // This uses the output
        output -> { drive.setRaw(output/180, DrivetrainConstants.kStopMotors);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drive;
    m_visionSubsystem = vision;
    addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
    // Configure additional PID options by calling `getController` here.
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
