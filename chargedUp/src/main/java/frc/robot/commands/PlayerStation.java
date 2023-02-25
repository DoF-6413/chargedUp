// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PlayerStation extends CommandBase {
  /** Creates a new PlayerStation. */
  VisionSubsystem m_visionSubsystem;
  ArmSubsystem m_armSubsysystem;

  public PlayerStation(ArmSubsystem arm, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsysystem = arm;
    m_visionSubsystem = vision;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    return atStation() ? m_armSubsysystem
  }

  public boolean atStation(){
    return m_visionSubsystem.getBestFiducial() == 4 || m_visionSubsystem.getBestFiducial() == 5 ? true: false;
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
