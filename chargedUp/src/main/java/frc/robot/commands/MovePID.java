// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;


/** THIS FILE IS FOR PID TUNING/TESTING. IF MOVING IN AUTO, PREFERABLY USE PATH PLANNER */
public class MovePID extends PIDCommand {
  /** Creates a new MovePID. */
  public MovePID(DrivetrainSubsystem drive, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(
          DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD
        ),
        // This should return the measurement
        () -> drive.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> { drive.setRaw(output, 0);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    // Configure additional PID options by calling `getController` here.
    System.out.println("ISRUNNING IS RUNNING IS RUNNING");
    getController().setTolerance(DrivetrainConstants.kMoveTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
