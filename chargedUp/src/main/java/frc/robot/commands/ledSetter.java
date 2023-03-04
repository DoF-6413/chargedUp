// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ledsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ledSetter extends CommandBase {
  /** Creates a new ledSetter. */

  private final ledsSubsystem m_LedsSubsystem;
  public ledSetter(ledsSubsystem ledSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LedsSubsystem = ledSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("running");
    //    m_LedsSubsystem.setLeds(-.99);
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
