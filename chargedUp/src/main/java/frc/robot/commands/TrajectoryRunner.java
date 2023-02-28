// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryRunner extends CommandBase {
  /** Creates a new TrajectoryRunner. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Timer m_timer;
  private Trajectory m_trajectory;
  private final RamseteController m_ramseteController = new RamseteController();
  
  public TrajectoryRunner(DrivetrainSubsystem drive, Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drive;
    m_trajectory = traj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // DrivetrainSubsystem.m_field2d.setRobotPose(m_trajectory.getInitialPose());
    m_drivetrainSubsystem.resetOdometry(m_trajectory.getInitialPose());
    
     // if(RobotBase.isSimulation()){
    //   RobotContainer.getDrive().setRobotFromFieldPose();
    // }
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    
    m_timer = new Timer();
    m_timer.start();

    // Reset the drivetrain's odometry to the starting pose of the trajectory. thisd should only happen if it is the first autonomus routine ran
    // RobotContainer.m_drivetrainSubsystem.resetOdometry(m_Trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.updateOdometry();

    // // Update robot position on Field2d.
    m_drivetrainSubsystem.setRobotFromFieldPose();

      // Get the desired pose from the trajectory.
      var desiredPose = m_trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(m_drivetrainSubsystem.getPose(), desiredPose);
      // Set the linear and angular speeds.

      // m_drivetrainSubsystem.setRaw(5, 0);
      // System.out.println("red chassis speed " + refChassisSpeeds);
      m_drivetrainSubsystem.setRaw(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrainSubsystem.setRaw(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_trajectory.sample(m_timer.get()) == m_trajectory.sample(m_trajectory.getTotalTimeSeconds()) ? true : false;
  }
}
