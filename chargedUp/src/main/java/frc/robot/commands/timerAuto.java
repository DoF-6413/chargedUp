// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class timerAuto extends CommandBase {
  DrivetrainSubsystem m_DrivetrainSubsystem;
  Timer timer = new Timer();
  /** Creates a new timerAuto. */
  public timerAuto(DrivetrainSubsystem drive) {
 
    m_DrivetrainSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DrivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_DrivetrainSubsystem.setRaw(0.25, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >=5 ;
  }
}
