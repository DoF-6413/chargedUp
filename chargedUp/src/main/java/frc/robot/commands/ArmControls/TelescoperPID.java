// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescoperPID extends PIDCommand {
  /** Creates a new TelescoperPID. */
  private ArmSubsystem m_armSubsystem;
  public TelescoperPID(ArmSubsystem arm, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(ArmConstants.kTelescoperP, ArmConstants.kTelescoperI, ArmConstants.kTelescoperD),
        // This should return the measurement
        () -> arm.getTelescoperPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> { arm.spinTelescopingMotor(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
    addRequirements(m_armSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(ArmConstants.kTelescoperTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
