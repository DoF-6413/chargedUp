// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimator;

public class TrajectoryRunner extends CommandBase {
  /** Creates a new TrajectoryRunner. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private PoseEstimator m_PoseEstimator;
  private Timer m_timer;
  private Trajectory m_trajectory;
  private Boolean m_isFirstPath;
  private RamseteCommand m_ramseteCommand;
  
  public TrajectoryRunner(DrivetrainSubsystem drive, PoseEstimator poseEstimator, Trajectory traj, Boolean isfirstPath) {
    /*Trajectory runner takes a drive subsystem and a trajectory, and a boolean to make the robot follow a certain path. 
    If the boolean is set to true, we reset the position*/
    m_drivetrainSubsystem = drive;
    m_trajectory = traj;
    m_isFirstPath = isfirstPath;
    m_ramseteCommand = new RamseteCommand(
      m_trajectory, 
      poseEstimator::getcurrentPose, 
      new RamseteController(
        DrivetrainConstants.kRamseteB, 
        DrivetrainConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(
        DrivetrainConstants.ksVolts, 
        DrivetrainConstants.kvVoltSecondPerMeter,
        DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
        DrivetrainConstants.kinematics, 
        m_drivetrainSubsystem::getWheelSpeeds, 
      new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
      new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
      m_drivetrainSubsystem::tankDrive, 
      m_drivetrainSubsystem);
    addRequirements(drive);
    m_PoseEstimator = poseEstimator;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if(m_isFirstPath == true){
    PoseEstimator.resetPose(m_trajectory.getInitialPose());
    }
    
    m_PoseEstimator.setRobotFromFieldPose();
    m_ramseteCommand.schedule();
    m_timer = new Timer();
    m_timer.start();
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    m_PoseEstimator.getcurrentPose();

//       // Get the desired pose from the trajectory.
//       var desiredPose = m_trajectory.sample(m_timer.get());
//       SmartDashboard.putString("erised", desiredPose.toString());
// SmartDashboard.putString("get pose", m_PoseEstimator.getcurrentPose().toString());
//       // Get the reference chassis speeds from the Ramsete controller.
//       var refChassisSpeeds = m_ramseteController.calculate(m_PoseEstimator.getcurrentPose(), desiredPose);
//       // Set the linear and angular speeds.

//       m_drivetrainSubsystem.setRaw(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_trajectory.sample(m_timer.get()) == m_trajectory.sample(m_trajectory.getTotalTimeSeconds()) ? true : false;
  }
}
