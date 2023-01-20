// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainPID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnPID extends PIDCommand {
  /** Creates a new TurnPID. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  GyroSubsystem m_gyroSubsystem;
  public TurnPID(DrivetrainSubsystem drive, GyroSubsystem gyro, double targetAngle) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.kTurnP, DrivetrainConstants.kTurnI, DrivetrainConstants.kTurnD),
        // This should return the measurement
        () -> gyro.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> { drive.setRaw(DrivetrainConstants.kStopMotors, output/180); //Divides by 180 to make output in between -1 and 1
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here
    m_drivetrainSubsystem = drive;
    m_gyroSubsystem = gyro;
    addRequirements(m_drivetrainSubsystem, m_gyroSubsystem);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(DrivetrainConstants.kMoveTolerance);
  }

  @Override
  public void initialize() {
    m_gyroSubsystem.resetYaw();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getController().atSetpoint() == true);
  }
}
