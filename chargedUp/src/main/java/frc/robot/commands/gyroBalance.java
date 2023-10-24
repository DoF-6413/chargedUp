// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class gyroBalance extends CommandBase {
  /** Creates a new gyroBalance. */
  private GyroSubsystem m_gyroSubsystem;
  private DrivetrainSubsystem m_drivetrainSubsystem;
  public gyroBalance(GyroSubsystem gyro, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyroSubsystem = gyro;
    m_drivetrainSubsystem = drive;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setRaw(0, 0);
    m_drivetrainSubsystem.switchIdleMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_gyroSubsystem.getRoll() > -10 && m_gyroSubsystem.getRoll() < 10){
      
      m_drivetrainSubsystem.setRaw(0, 0);
    }
     else if(m_gyroSubsystem.getRoll() > 0.5){
      //Change to negative
    m_drivetrainSubsystem.setRaw(0.35, 0);
    } else if (m_gyroSubsystem.getRoll() < 0){
      //change to positive
      m_drivetrainSubsystem.setRaw(-0.35, 0);
    } else {
      m_drivetrainSubsystem.setRaw(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(0, 0);
    // m_drivetrainSubsystem.switchIdleMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (m_gyroSubsystem.getRoll() > -1 && m_gyroSubsystem.getRoll() < 1) ? true : false;
    return false;
  }
}
