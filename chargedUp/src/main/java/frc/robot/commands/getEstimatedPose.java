// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class getEstimatedPose extends CommandBase {
  /** Creates a new getEstimatedPose. */
  private DrivetrainSubsystem m_drive;
  private GyroSubsystem m_gyro;
  private VisionSubsystem m_vision;
  private static PoseEstimator m_PoseEstimator;
  private boolean finishVar = false;
  public getEstimatedPose(GyroSubsystem gyro,DrivetrainSubsystem drive, VisionSubsystem vision, PoseEstimator pose ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_gyro = gyro;
    m_vision = vision;
    m_PoseEstimator = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PoseEstimator.getcurrentPose();
  m_drive.resetOdometry(m_PoseEstimator.getcurrentPose());
 SmartDashboard.putString("derivedposet", m_PoseEstimator.getcurrentPose().toString());
 finishVar = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishVar;
  }
}
