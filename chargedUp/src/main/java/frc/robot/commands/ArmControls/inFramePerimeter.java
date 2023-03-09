// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

public class inFramePerimeter extends CommandBase {
  /** Creates a new inFramePerimeter. */
  private TelescoperSubsystem m_telescoperSubsystem;
  private ArmSubsystem m_armSubsystem;
  public inFramePerimeter(TelescoperSubsystem telescope, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescoperSubsystem = telescope;
    m_armSubsystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_armSubsystem.getRotationPosition() < 35) && (m_armSubsystem.getRotationPosition() > -35 )){
      System.out.println("Inside Frame Perimeter");
    } else {
      System.out.println("Outside Frame Perimeter");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
