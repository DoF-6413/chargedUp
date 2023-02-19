// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class targetFinding extends CommandBase {
  /** Creates a new targetFinding. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  public targetFinding(DrivetrainSubsystem drive, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drive;
    m_visionSubsystem = vision;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setRaw(0, 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_visionSubsystem.isHuge() != true){
      m_drivetrainSubsystem.setRaw(0.5, 0);
    }
    System.out.println(m_visionSubsystem.seeTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // replace when enabled
    // return m_drivetrainSubsystem.getPosition() > 5000;
  }
}
