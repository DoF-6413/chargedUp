// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TelescoperReset extends CommandBase {
  /** Creates a new TelescoperReset. */
  ArmSubsystem m_armSubsystem;
  public TelescoperReset(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.telescoperCurrentLimit(10, 20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.spinTelescopingMotor(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.spinTelescopingMotor(0);
    m_armSubsystem.resetTelescoperPosition();
    m_armSubsystem.telescoperCurrentLimit(ArmConstants.kTelescoperContinuousCurrent, ArmConstants.kTelescoperPeakCurrent);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.telecoperCurrent() >= 5;
  }
}
