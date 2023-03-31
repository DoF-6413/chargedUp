// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class getEstimatedPose extends CommandBase {
  /** Creates a new getEstimatedPose. */
  PoseEstimator oseEstimator;
  public getEstimatedPose(GyroSubsystem gyro,DrivetrainSubsystem drive, VisionSubsystem vision ) {
    // Use addRequirements() here to declare subsystem dependencies.
    new PoseEstimator(
      gyro,drive, vision, DrivetrainConstants.kinematics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  PoseEstimator.getcurrentPose();

 SmartDashboard.putString("derivedposet", PoseEstimator.getcurrentPose().toString());
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
