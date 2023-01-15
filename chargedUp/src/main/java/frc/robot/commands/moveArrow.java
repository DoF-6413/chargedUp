// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class moveArrow extends CommandBase {
  /** Creates a new moveArrow. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private double m_startPosition;
  private int m_fichialID;
  public moveArrow(DrivetrainSubsystem drive, VisionSubsystem vision ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drive;
    m_visionSubsystem = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setRaw(0, 0);
    m_startPosition = m_drivetrainSubsystem.getPosition();
    m_fichialID = m_visionSubsystem.getBestFiducial();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets Fiducial ID
    if(m_startPosition > Constants.FiducialConstants.kArrowValues[m_fichialID]){
      m_drivetrainSubsystem.setRaw(-0.3, 0);
    } else if (m_startPosition < Constants.FiducialConstants.kArrowValues[m_fichialID]){
      m_drivetrainSubsystem.setRaw(0.3, 0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  
  @Override
  public boolean isFinished() {
    return (m_startPosition > Constants.FiducialConstants.kArrowValues[m_fichialID]) ? 
    m_drivetrainSubsystem.getPosition() < m_startPosition : m_drivetrainSubsystem.getPosition() > m_startPosition;
  }
}
