// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem; 
import frc.robot.subsystems.DrivetrainSubsystem;

public class getPoseAprilTag extends CommandBase {
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
private final VisionSubsystem m_VisionSubsystem;

  /** Creates a new getPoseAprilTag. */
  public getPoseAprilTag(DrivetrainSubsystem drive, VisionSubsystem vision) {
    m_DrivetrainSubsystem = drive;
    m_VisionSubsystem = vision;
   addRequirements(m_DrivetrainSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab apriltag 
    double yaw = m_VisionSubsystem.getTargetYaw();
    Transform3d distanceTo = m_VisionSubsystem.distanceFinder(); 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
