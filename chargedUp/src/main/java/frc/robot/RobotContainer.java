// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.targetFinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.colorSensor;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.EndEffectorRunner;
import frc.robot.commands.ArmControls.RotationPID;
// import frc.robot.commands.ArmControls.TelescoperConditional;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.commands.Autos.ScoreCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.colorSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  
  private final TelescoperSubsystem m_telescoperSubsystem = new TelescoperSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();

  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  private final colorSensor m_colorSensorSubsystem = new colorSensor();

  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("Newish path", new PathConstraints(2, 0.8));
  PathPlannerTrajectory getOntoChargingStation = PathPlanner.loadPath("GetOntoCSJanky", new PathConstraints(2, 0.8));

  PathPlannerTrajectory kscoreWithEvents = PathPlanner.loadPath("scoreWithEvents", new PathConstraints(1.5, 0.3));
  Trajectory m_Trajectory = 
  TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public final static CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * 
   */
  public RobotContainer() {

    HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("marker1", new RunCommand(() -> System.out.println("leaving community")));
eventMap.put("marker2", new RunCommand(() -> System.out.println("hit marker 2")));
    // Configure the trigger bindings
    m_chooser.setDefaultOption("Test Path", new TrajectoryRunner(m_drivetrainSubsystem, testPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Newish Path", new TrajectoryRunner(m_drivetrainSubsystem, newishPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Get Onto Charging Station", new TrajectoryRunner(m_drivetrainSubsystem, getOntoChargingStation.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Score Cone", new ScoreCone(m_armSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_drivetrainSubsystem));
    m_chooser.addOption("follow path events", new FollowPathWithEvents(
      new TrajectoryRunner(m_drivetrainSubsystem, kscoreWithEvents.relativeTo(m_drivetrainSubsystem.getPose()), true),
      kscoreWithEvents.getMarkers(),
      eventMap
  ));
    SmartDashboard.putData("m_chooser", m_chooser);
    configureBindings();
    defaultCommands();
      
  }

  public void defaultCommands(){
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(-m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrainSubsystem));

     m_armSubsystem.setDefaultCommand(new RunCommand(() -> m_armSubsystem.spinRotationMotors(-m_auxController.getLeftY()), m_armSubsystem));
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_auxController.back().onTrue(new TelescoperReset(m_telescoperSubsystem));
        // //This runs Endeffector to Collect Cube
        m_auxController.start().onTrue(new InstantCommand(()-> m_armSubsystem.resetRotationPosition()));
        //This runs Endeffector to Collect Cone
        // m_auxController.a().
        // onTrue(new InstantCommand(()-> m_armSubsystem.resetRotationPosition()));
        m_auxController.a().
        onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));
        
        m_auxController.b().
        onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.07)));

        // // This runs Endeffector to eject game peices
        m_auxController.rightTrigger().
        onTrue(new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(-0.2)))
        .onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));
        
        // m_auxController.y()
        // .onTrue(
        //   new ConditionalCommand(
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   new TelescoperPID(m_telescoperSubsystem, 150), 
        //   () ->  m_armSubsystem.isInFramePerimeter()
        //   ))
        // .onFalse(
        //   new ConditionalCommand(
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   () -> m_armSubsystem.isInFramePerimeter()
        //   ));

        //   m_auxController.x()
        // .onTrue(
        //   new ConditionalCommand(
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   new TelescoperPID(m_telescoperSubsystem, 30), 
        //   () ->  m_armSubsystem.isInFramePerimeter()
        //   ))
        // .onFalse(
        //   new ConditionalCommand(
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   new TelescoperPID(m_telescoperSubsystem, 0), 
        //   () -> m_armSubsystem.isInFramePerimeter()
        //   ));

        m_auxController.y().onTrue(new RotationPID(m_armSubsystem, 90));
        m_auxController.x().onTrue(new RotationPID(m_armSubsystem, -90));
        m_auxController.leftBumper().onTrue(new RotationPID(m_armSubsystem, 0));

        // m_auxController.y()
        // .onTrue(
        //   new ConditionalCommand(
        //   new InstantCommand(() -> System.out.println("Pressed Y and is True")), 
        //   new InstantCommand(() -> System.out.println("Pressed Y and is False")), 
        //   () ->  m_armSubsystem.isInFramePerimeter()
        //   ))
        // .onFalse(
        //   new ConditionalCommand(
        //   new InstantCommand(() -> System.out.println("Released Y and is True")), 
        //   new InstantCommand(() -> System.out.println("Released Y and is False")), 
        //   () -> m_armSubsystem.isInFramePerimeter()
        //   ));

        // m_auxController.x().
        // onTrue(new TelescoperPID(m_telescoperSubsystem, 50));


  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }
}
