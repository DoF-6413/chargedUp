// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToCone extends CommandBase {
  /** Creates a new locateCube. */
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final VisionSubsystem m_VisionSubsystem;
  private static double coneX;
  private static double coneY;
  private static double turnVar;

  public DriveToCone(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    /*
     * This is a work in progress that can be used for future actions. Requires
     * further development.
     */
    // Use addRequirements() here to declare subsystem dependencies.
    m_VisionSubsystem = visionSubsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    turnVar = 0.0;
    addRequirements(drivetrainSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.setRaw(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    System.out.println("Running");
    if (m_VisionSubsystem.seeTarget() == true) {
      coneX = m_VisionSubsystem.photonResult().getBestTarget().getMinAreaRectCorners().get(0).x;
      coneY = m_VisionSubsystem.photonResult().getBestTarget().getMinAreaRectCorners().get(0).y;
      
      SmartDashboard.putNumber("getXvalue", m_VisionSubsystem.photonResult().getBestTarget().getMinAreaRectCorners().get(0).x);
      SmartDashboard.putNumber("getYvalue", m_VisionSubsystem.photonResult().getBestTarget().getMinAreaRectCorners().get(0).y);
      if (coneX < 30.0) {
        SmartDashboard.putString("Which Direction?", "Left");
        m_DrivetrainSubsystem.setRaw(0.65, 0.3);
      } else if (coneX > 50.0) {
        m_DrivetrainSubsystem.setRaw(0.65, -0.3);
        SmartDashboard.putString("Which Direction?", "Right");
      } else {
        m_DrivetrainSubsystem.setRaw(0.65, 0.0);
        SmartDashboard.putString("Which Direction?", "Straight");
      }

    } else {

      m_DrivetrainSubsystem.setRaw(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
