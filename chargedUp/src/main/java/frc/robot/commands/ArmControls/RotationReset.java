// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RotationReset extends CommandBase {
  /** Creates a new RotationReset. */
  private static ArmSubsystem m_armSubsystem;
  public RotationReset(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.getPotentiometer() > 0.5){
      m_armSubsystem.spinRotationMotors(-0.2);
    } else if (m_armSubsystem.getPotentiometer() < -0.5){
      m_armSubsystem.spinRotationMotors(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopRotationMotors();
    m_armSubsystem.resetRotationPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_armSubsystem.getPotentiometer() < 1 && m_armSubsystem.getPotentiometer() > -1) == true) ? true : false;
  }
}
