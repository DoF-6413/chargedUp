// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

public class RotationReset extends CommandBase {
  /** Creates a new RotationReset. */
  private static ArmPIDSubsystem m_armSubsystem;
  private static TelescoperSubsystem m_telescoperSubsystem;
  public RotationReset(ArmPIDSubsystem arm, TelescoperSubsystem telescoper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
    m_telescoperSubsystem = telescoper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new TelescoperReset(m_telescoperSubsystem).schedule();
    m_armSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.getPotentiometer() > 1){
      m_armSubsystem.setVoltage(-1);
    } else if (m_armSubsystem.getPotentiometer() < -1){
      m_armSubsystem.setVoltage(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.resetRotationPosition();
    m_armSubsystem.enable();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(m_armSubsystem.getPotentiometer()) < 1 ) == true) ? true : false;
  }
}
