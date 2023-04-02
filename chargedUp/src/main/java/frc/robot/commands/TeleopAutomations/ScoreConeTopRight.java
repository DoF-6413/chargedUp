// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopAutomations;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeTopRight extends SequentialCommandGroup {
  /** Creates a new ScoreConeTopRight. */
  public ScoreConeTopRight(ArmSubsystem arm, DrivetrainSubsystem drive, TelescoperSubsystem telescoper, EndEffectorSubsystem endeffector) {
    // Add your commands in the addCommands() call, e.g.
    PathPlannerTrajectory kgridRightwithevents = PathPlanner.loadPath("gridRight", new PathConstraints(1, 1));
    // addCommands(new FooCommand(), new BarCommand());
    HashMap<String, Command> evenMapGridRight = new HashMap<>();
    evenMapGridRight.put("armOut", new PositionHigh(arm, telescoper, endeffector));
    
    addCommands(
      //on the fly generating

      //follow path with events 
      
      //scores and puts arm back inn 
      new PlaceHigh(arm, telescoper, endeffector)
    
    );
  }
}
