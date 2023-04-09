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
import edu.wpi.first.wpilibj.RobotBase;
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
  // private RamseteCommand
   private RamseteController m_ramseteCommand;
  private double m_desiredX, m_desiredY, m_desiredRot;
  private double m_currX, m_currY, m_currRot;

  
  public TrajectoryRunner(DrivetrainSubsystem drive, PoseEstimator poseEstimator, Trajectory traj, Boolean isfirstPath) {
    /*Trajectory runner takes a drive subsystem and a trajectory, and a boolean to make the robot follow a certain path. 
    If the boolean is set to true, we reset the position*/
    m_drivetrainSubsystem = drive;
    m_trajectory = traj;
    m_isFirstPath = isfirstPath;
    // m_ramseteCommand = new RamseteCommand(
    //   m_trajectory, 
    //   poseEstimator::getcurrentPose, 
    m_ramseteCommand =
      new RamseteController(
        DrivetrainConstants.kRamseteB, 
        DrivetrainConstants.kRamseteZeta)
        ;
        // , 
      // new SimpleMotorFeedforward(
      //   DrivetrainConstants.ksVolts, 
      //   DrivetrainConstants.kvVoltSecondPerMeter,
      //   DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
      //   DrivetrainConstants.kinematics, 
      //   m_drivetrainSubsystem::getWheelSpeeds, 
      // new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
      // new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
      // m_drivetrainSubsystem::tankDrive, 
      // m_drivetrainSubsystem, poseEstimator);
    addRequirements(drive, poseEstimator);

    m_PoseEstimator = poseEstimator;
    m_desiredX = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getX();
    m_desiredY = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getY();
    m_desiredRot = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees();
    m_currX = m_PoseEstimator.getcurrentPose().getX();
    m_currY = m_PoseEstimator.getcurrentPose().getY();
    m_currRot = m_PoseEstimator.getcurrentPose().getRotation().getDegrees();


  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if(m_isFirstPath == true){
    PoseEstimator.resetPose(m_trajectory.getInitialPose());
    }
    
    m_PoseEstimator.setRobotFromFieldPose();
    m_timer = new Timer();
    m_timer.start();
    // m_ramseteCommand.schedule();
    if( RobotBase.isReal()){
    PoseEstimator.m_field2d.getObject("traj").setTrajectory(m_trajectory);
    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

      // Get the desired pose from the trajectory.
      var desiredPose = m_trajectory.sample(m_timer.get());
      SmartDashboard.putString("erised", desiredPose.toString());
SmartDashboard.putString("get pose", m_PoseEstimator.getcurrentPose().toString());
      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteCommand.calculate(m_PoseEstimator.getcurrentPose(), desiredPose);
      // Set the linear and angular speeds.

      m_drivetrainSubsystem.setRaw(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean terminate = 
    ( Math.abs( m_currX - m_desiredX ) <= 0.5)
    &&( Math.abs( m_currY - m_desiredY ) <= 0.5)
    &&( Math.abs( m_currRot - m_desiredRot ) <= 0.2);
    // return terminate;
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
