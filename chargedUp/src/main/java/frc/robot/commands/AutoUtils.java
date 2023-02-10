// maybe we need this
// package frc.robot.commands.Autonomous;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
// Todo:set autoconstants
// import frc.robot.Constants.AutoConstants;
// Todo:set drive constants
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.*;

// todo: import differential drivetrain subsystem
import frc.robot.subsystems.DrivetrainSubsystem; 


public class AutoUtils {
    // Default Constants
    private final static PathConstraints m_defaultConfig = new PathConstraints(
        AutoConstants.MAX_VELOCITY_PERCENT_OUTPUT, AutoConstants.MAX_ACCELERATION_PERCENT_OUTPUT);
        

    private static PathPlannerTrajectory m_defaultAutoGen = PathPlanner.loadPath("firstPath", m_defaultConfig);


    //Default getters
    public static Command getDefaultTrajectory(DrivetrainSubsystem drivetrain) {
        return new SequentialCommandGroup(

                new PPRamseteCommand(
    m_defaultAutoGen, 
    drivetrain::getPose, // Pose supplier
    new RamseteController(),
    new SimpleMotorFeedforward(1,1, 1),
    Constants.DrivetrainConstants.kinematics, // DifferentialDriveKinematics
    drivetrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
    new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
    drivetrain::outputVolts, // Voltage biconsumer
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    drivetrain // Requires this drive subsystem
)
        );
    }

    public static FollowPathWithEvents getPathWithEvents(DrivetrainSubsystem drivetrain) {
    HashMap<String, Command> defaultEventMap = new HashMap<>();
    defaultEventMap.put("Event 1", new PrintCommand("Marker 1"));
    defaultEventMap.put("event 2", new PrintCommand("Marker 2"));
    return new FollowPathWithEvents(getDefaultTrajectory(drivetrain), m_defaultAutoGen.getMarkers(), defaultEventMap);
    }

    public static Command getAutoRoutine(PathPlannerTrajectory traj, DrivetrainSubsystem drivetrain){
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPose(m_defaultAutoGen.getInitialHolonomicPose())),
// Todo: change to ramsette controller
new PPRamseteCommand(
    traj, 
    this::getPose, // Pose supplier
    new RamseteController(),
    new SimpleMotorFeedforward(KS, KV, KA),
    this.kinematics, // DifferentialDriveKinematics
    this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
    new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
    this::outputVolts, // Voltage biconsumer
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    this // Requires this drive subsystem
)
        );
    }
    
    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, HashMap<String, Command> events, DrivetrainSubsystem drivetrain) {
        return new FollowPathWithEvents(getAutoRoutine(traj, drivetrain), traj.getMarkers(), events);
    }
}