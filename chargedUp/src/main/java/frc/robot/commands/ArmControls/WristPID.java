// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristPID extends PIDCommand {
  /** Creates a new WristPID. */
  private WristSubsystem m_wristSubsystem;
  public WristPID(WristSubsystem wrist, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(WristConstants.kWristP, WristConstants.kWristI, WristConstants.kWristD),
        // This should return the measurement
        () -> wrist.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> { wrist.spinWrist(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.

    m_wristSubsystem = wrist;
    addRequirements(m_wristSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(WristConstants.kWristTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
