// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MultiPieceAuto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.Autos.BackingOutArm;
import frc.robot.commands.Autos.ScoreCone;
import frc.robot.commands.Autos.ScoreHigh;
import frc.robot.commands.TeleopAutomations.BackIn;
import frc.robot.commands.TeleopAutomations.PositionPickUp;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneAndAHalf extends SequentialCommandGroup {
  /** Creates a new OneAndAHalf. */
  public OneAndAHalf(DrivetrainSubsystem drive, ArmPIDSubsystem arm, TelescoperSubsystem telescoper,
      EndEffectorSubsystem NEfector, PoseEstimator pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory kPickUp = PathPlanner.loadPath("OneAndAHalf", new PathConstraints(2, 0.7));
    HashMap<String, Command> eventBringArmIn = new HashMap<>();
    eventBringArmIn.put("BringInArm", new ReturnArm(arm, telescoper, NEfector, drive));
    eventBringArmIn.put("PickUp", new GroundPickUp(telescoper, arm, NEfector));

      PathPlannerTrajectory.transformTrajectoryForAlliance(kPickUp, DriverStation.getAlliance());
      System.out.println(DriverStation.getAlliance());
  

    
    addCommands(
        new ScoreLeaveArm(arm, telescoper, NEfector, drive),
        new FollowPathWithEvents(
            new TrajectoryRunner(drive, pose, () -> kPickUp.relativeTo(pose.getcurrentPose()), true),
            kPickUp.getMarkers(),
            eventBringArmIn)
    );
  }
}
