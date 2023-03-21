// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristReset extends CommandBase {
  /** Creates a new WristReset. */
  private final WristSubsystem m_wristSubsystem;
  public WristReset(WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wristSubsystem = wrist;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wristSubsystem.setCurrentLimit(10);
    m_wristSubsystem.stopWrist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wristSubsystem.spinWrist(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_wristSubsystem.setCurrentLimit(10);
    m_wristSubsystem.stopWrist();
    m_wristSubsystem.resetWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wristSubsystem.getOutputCurrent() > 9 ? true: false;
  }
}
