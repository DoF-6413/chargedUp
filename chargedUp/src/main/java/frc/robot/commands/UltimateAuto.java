// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.Constants.TelescoperConstants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmControls.*;
import frc.robot.commands.Autos.PickUpGround;
import frc.robot.commands.Autos.ScoreCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UltimateAuto extends SequentialCommandGroup {
  private TelescoperSubsystem telescoper;
  private EndEffectorSubsystem endEffector;
  private ArmSubsystem arm;
  private DrivetrainSubsystem drive;
  PathPlannerTrajectory kscorePickUp = PathPlanner.loadPath("ScorePickUp", new PathConstraints(1.5, .8));
  // PathPlannerTrajectory kPickUpRun = PathPlanner.loadPath("PickUpRun", new PathConstraints(1.5, .8));
  /** Creates a new UltimateAuto. */
  public UltimateAuto(DrivetrainSubsystem  m_drivetrainSubsystem, ArmSubsystem m_armSubsystem, TelescoperSubsystem m_telescoperSubsystem, EndEffectorSubsystem m_endEffectorSubsystem) {
    telescoper = m_telescoperSubsystem;
    endEffector = m_endEffectorSubsystem;
    arm = m_armSubsystem;
    drive =  m_drivetrainSubsystem;
    addRequirements(telescoper, endEffector, arm, drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Score cone
    new ScoreCone(arm, telescoper, endEffector, drive),
      // trajectoryRunner that gets you to the cone
    new TrajectoryRunner(drive, kscorePickUp.relativeTo(drive.getPose()), true),
      // pikup ground
    new PickUpGround(arm, telescoper, endEffector)
      //trajectoryRunner that get you to score 
    // new TrajectoryRunner(drive, kPickUpRun.relativeTo(drive.getPose()), true),
     //Score Cone again
    // new ScoreCone(arm, telescoper, endEffector, drive)
    );
  }
}
