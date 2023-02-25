// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveCommand extends CommandBase {
  /** Creates a new MoveCommand. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private double m_setpoint;
  private double m_speed;

  /*Requires drivetrain subystem, setpoint in feet, and a positive speed 0-1 (0 being 0% 1 being 100%) */
  public MoveCommand(DrivetrainSubsystem drive, double setpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drive;
    m_setpoint = setpoint;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.resetPosition();
  }

  public boolean isForward(){
    return m_setpoint > 0 ? true : false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(isForward()){
      m_drivetrainSubsystem.setRaw(m_speed, 0);
  //   } else
  //     m_drivetrainSubsystem.setRaw(-m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("setpoint", m_setpoint);
    return 
    // isForward() ? 
    m_setpoint <= m_drivetrainSubsystem.getPosition();
    //  : m_setpoint >= m_drivetrainSubsystem.getPosition();
  }
}
