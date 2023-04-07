// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMovePickupScore extends SequentialCommandGroup {
  /** Creates a new ScoreMovePickupScore. */
  PathPlannerTrajectory kPickUp = PathPlanner.loadPath("PickUp", new PathConstraints(1, .8));
  PathPlannerTrajectory kPlace = PathPlanner.loadPath("Place", new PathConstraints(1, .8));
  // PathPlannerTrajectory kPickUp = PathPlanner.loadPath("PickUp", new PathConstraints(1, .8));
  PathPlannerTrajectory kPlace2 = PathPlanner.loadPath("TestingRamsete", new PathConstraints(2, 4));
  public ScoreMovePickupScore(DrivetrainSubsystem drive, ArmSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem NEfector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    HashMap<String, Command> eventBringArmIn = new HashMap<>();
    eventBringArmIn.put("BringInArm", new BackingOutArm(arm, telescoper, NEfector));
    
    addCommands(
    new ScoreCone(arm, telescoper, NEfector, drive),
    new FollowPathWithEvents(
      new TrajectoryRunner(drive, kPickUp.relativeTo(drive.getPose()), true),
      kPickUp.getMarkers(),
      eventBringArmIn
      ),
    new GroundPickUp(telescoper, arm, NEfector),
    new TrajectoryRunner(drive, kPlace.relativeTo(drive.getPose()), true),
    new ScoreHigh(arm, telescoper, NEfector, drive)
    );
  }
}
