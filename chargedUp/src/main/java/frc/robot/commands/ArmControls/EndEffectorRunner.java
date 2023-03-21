// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorRunner extends CommandBase {
  /** Creates a new EndEffectorRunner. */
  EndEffectorSubsystem m_endEffexEffectorSubsystem;
  double m_speed;
  double m_time;
  Timer m_timer;

  public EndEffectorRunner(EndEffectorSubsystem NEfctr, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies
    m_endEffexEffectorSubsystem = NEfctr;
    m_speed = speed;
    m_timer = new Timer();
    m_time = time;
    addRequirements(NEfctr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffexEffectorSubsystem.stopEndEffector();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endEffexEffectorSubsystem.spinEndEffector(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffexEffectorSubsystem.stopEndEffector();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_time;
  }
}
