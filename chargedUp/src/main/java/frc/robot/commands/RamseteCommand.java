// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;




public class RamseteCommand extends CommandBase {
  Timer m_timer;
  Trajectory m_Trajectory;
  
  


  
  private final RamseteController m_ramseteController = new RamseteController();

  /** Creates a new RamseteCommand. */
  public RamseteCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() < m_Trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = m_Trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(RobotContainer.m_drivetrainSubsystem.getPose(), desiredPose);

      // Set the linear and angular speeds.
      RobotContainer.m_drivetrainSubsystem.setRaw(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      RobotContainer.m_drivetrainSubsystem.setRaw(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ternary that returns true if the current stae equals the final state of trajectory.
    return (m_Trajectory.sample(m_Trajectory.getTotalTimeSeconds()) == m_Trajectory.sample(m_timer.get())) ? true: false;
  }
}
