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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public static Command m_eventMapAction;
  public static int m_superCode;
  public static PathPlannerTrajectory m_chosenTraj;
 public static PathPlannerTrajectory goToGrid0 = PathPlanner.loadPath("grid0Right", new PathConstraints(1, 1));
 public static PathPlannerTrajectory goToGrid1 = PathPlanner.loadPath("grid1Right", new PathConstraints(1, 1));
 public static PathPlannerTrajectory goToGrid2 = PathPlanner.loadPath("grid2Right", new PathConstraints(1, 1));
 /** Creates a new command that scores a cone on the right grid mid right (Blue Alliance) */
  public ScoreConeRightGridMid(ArmSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endeffector, PoseEstimator poseEstimator, Command eventMapAction, Command finalAction, Integer supercode) {
    // Add your commands in the addCommands() call, e.g.
  
   
    // PathPlannerTrajectory goToGrid2 =
    // addCommands(new FooCommand(), new BarCommand());
    // HashMap<String, Command> eventMapGridRightMid = new HashMap<>();
    // eventMapGridRightMid.put("armout", getCommand());


m_eventMapAction = eventMapAction;
m_superCode = supercode;
    // PathPlannerTrajectory gridRightMidTraj = PathPlanner.generatePath(
    //   new PathConstraints(1, 1),
    //   new PathPoint(getChosenTraj(supercode).getInitialPose().getTranslation(), getChosenTraj(supercode).getInitialPose().getRotation()),
    //   new PathPoint(new Translation2d(3.35,7), new Rotation2d(1)));
    addCommands(
       //on the fly generating
      //  new TrajectoryRunner(drive, gridRightMidTraj.relativeTo(poseEstimator.getcurrentPose()), false),
       //follow path with events 
      //  new FollowPathWithEvents(
         new TrajectoryRunner(drive, getChosenTraj(supercode).relativeTo(poseEstimator.getcurrentPose()), false)
      //  gridRightMid.getMarkers(),
      //  eventMapGridRightMid
       );
      //  ,
       //scores and puts arm back in 
      // getFinalCommand(supercode)
    // );
  }

  public static PathPlannerTrajectory getChosenTraj(int routineCode){
    if(routineCode == 00){
      return goToGrid0;
    } else if(routineCode == 01){
      return goToGrid1;
    } else if(routineCode == 02){
      return goToGrid2;
    }
    return new PathPlannerTrajectory();
  }

  private static Command getFinalCommand(Integer routineCode){
// in here use the routine code to return the actual command you need to score a game piece
    Command gotCommand = (routineCode == 01) ? m_eventMapAction  : m_eventMapAction;
return gotCommand;
  }
}
