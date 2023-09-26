// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.Constants.TelescoperConstants;

public class TelescoperReset extends CommandBase {
  /** Creates a new TelescoperReset. */
  TelescoperSubsystem m_telescoperSubsystem;
  public TelescoperReset(TelescoperSubsystem telescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescoperSubsystem = telescope;
    addRequirements(m_telescoperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_telescoperSubsystem.telescoperCurrentLimit(30, 40);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescoperSubsystem.spinTelescopingMotor(-0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescoperSubsystem.spinTelescopingMotor(0);
    m_telescoperSubsystem.resetTelescoperPosition();
    m_telescoperSubsystem.telescoperCurrentLimit(TelescoperConstants.kTelescoperContinuousCurrent, TelescoperConstants.kTelescoperPeakCurrent);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescoperSubsystem.telecoperCurrent() >= 30;
  }
}
