// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MultiPieceAuto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TrajectoryRunner;
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
public class TwoPiecePPAuto extends SequentialCommandGroup {
  /** Creates a new TwoPiecePPAuto. */

  public TwoPiecePPAuto(DrivetrainSubsystem drive, ArmPIDSubsystem arm, TelescoperSubsystem telescoper, EndEffectorSubsystem endEffector, PoseEstimator pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPlannerTrajectory kTwoScore = PathPlanner.loadPath("TwoPieceFullPath", new PathConstraints(1, .6));
    HashMap<String,Command> eventTwoScore= new HashMap<>();
    eventTwoScore.put("BringInArm", new RunCommand(()-> System.out.print("BRINGING IN ARM")));
    // eventTwoScore.put("BringInArm", new ReturnArm(arm, telescoper, endEffector, drive));
    // eventTwoScore.put("PickUp", new PositionPickUp(telescoper, arm, endEffector));
    // eventTwoScore.put("GrabPiece", new BackIn(telescoper, arm));

    // eventTwoScore.put("PickUp", new RunCommand( ()-> System.out.print("PICKING UP CONE")));
    // eventTwoScore.put("GrabPiece", new RunCommand( ()-> System.out.print("GRABBING")));


    // PathPlannerTrajectory.transformTrajectoryForAlliance(kTwoScore, DriverStation.getAlliance());
      // System.out.println(DriverStation.getAlliance());

    addCommands(
      // new ScoreLeaveArm(arm, telescoper, endEffector, drive),
      new TrajectoryRunner(drive, pose, ()-> kTwoScore.relativeTo(pose.getcurrentPose()), true)
      // new ScoreHigh(arm, telescoper, endEffector, drive)
      
    );
  }
}
