// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.VisionSubsystem; 
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class locateCube extends CommandBase {
  /** Creates a new locateCube. */
private final DrivetrainSubsystem m_DrivetrainSubsystem;
private final VisionSubsystem m_VisionSubsystem;

  public locateCube( DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_VisionSubsystem = visionSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_VisionSubsystem.distanceFinder();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute()
  {
    // turn clockwise untill it sees a target 
    if ( m_VisionSubsystem.seeTarget() != true ){ 
      m_DrivetrainSubsystem.setRaw(0, .5);

      SmartDashboard.putString("running", "execute with target");

    }else if ( m_VisionSubsystem.seeTarget() == true ) {
      m_DrivetrainSubsystem.setRaw(-.5, 0);
    }
    // if not target turn clockwise until you do have a target 
    // if you do have a target center the target if target is right of your center point turn right if left of your center point turn left once hits center point drive straight
  

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
