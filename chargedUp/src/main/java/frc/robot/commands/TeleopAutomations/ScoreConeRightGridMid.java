// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeRightGridMid extends SequentialCommandGroup {
  /** Creates a new ScoreConeRightGridMid. */
  public ScoreConeRightGridMid(ArmSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endeffector, PoseEstimator poseEstimator) {
    // Add your commands in the addCommands() call, e.g.
    PathPlannerTrajectory gridRightMid = PathPlanner.loadPath("gridRight", new PathConstraints(1, 1));
    // addCommands(new FooCommand(), new BarCommand());
    HashMap<String, Command> eventMapGridRightMid = new HashMap<>();
    eventMapGridRightMid.put("armout", new PositionMid(telescoper, arm, endeffector));

    PathPlannerTrajectory gridRightMidTraj = PathPlanner.generatePath(
      new PathConstraints(1, 1),
      new PathPoint(gridRightMid.getInitialPose().getTranslation(), gridRightMid.getInitialPose().getRotation()),
      new PathPoint(new Translation2d(3.35,7), new Rotation2d(1)));
    addCommands(
       //on the fly generating
       new TrajectoryRunner(drive, gridRightMidTraj.relativeTo(poseEstimator.getcurrentPose()), false),
       //follow path with events 
       new FollowPathWithEvents(
         new TrajectoryRunner(drive, gridRightMid.relativeTo(poseEstimator.getcurrentPose()), false),
       gridRightMid.getMarkers(),
       eventMapGridRightMid
       ),
       //scores and puts arm back in
       new PlaceMid(arm, telescoper, endeffector)
    );
  }
}
