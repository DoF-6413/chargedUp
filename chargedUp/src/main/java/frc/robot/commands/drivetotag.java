// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class drivetotag extends CommandBase {


  /** Creates a new drivetotag. */
  Double m_setpoint;
private final DrivetrainSubsystem m_DrivetrainSubsystem;
private final VisionSubsystem m_VisionSubsystem;


public drivetotag( DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_setpoint = setpoint;
  m_DrivetrainSubsystem = drivetrainSubsystem;
  m_VisionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("execute X", m_VisionSubsystem.distanceFinder().getX());
    SmartDashboard.putNumber("set point", m_setpoint);
   if (m_VisionSubsystem.distanceFinder().getX() > m_setpoint){
    m_DrivetrainSubsystem.setRaw(.2, 0);
   }if(m_VisionSubsystem.distanceFinder().getX() < m_setpoint){
    m_DrivetrainSubsystem.setRaw(0, 0);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Todo return true is getX() is less than 0.3
    return false;
  }
}
