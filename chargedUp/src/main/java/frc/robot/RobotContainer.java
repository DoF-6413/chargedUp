// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.DrivetrainPID.MovePID;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drivetotag;
import frc.robot.commands.locateCube;
import frc.robot.commands.targetFinding;
import frc.robot.subsystems.DriveSimSub;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonActionListener;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  
  // PathPlannerTrajectory firstPath = PathPlanner.loadPath("firstPath", new PathConstraints(4, 3));
  PathPlannerTrajectory firstPath = PathPlanner.loadPath("firstPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("Newish path", new PathConstraints(2, 0.8));
  
  PathPlannerTrajectory newerishPath = PathPlanner.loadPath("NewerishPath", new PathConstraints(2, 0.8));

  Trajectory m_Trajectory = 
  // firstPath.relativeTo(firstPath.getInitialPose());
  TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
  
  // list of autos
  private final Command m_autoScore = new AutoScore2();
  private final Command m_moveForward = 
      new MovePID(m_drivetrainSubsystem, AutoConstants.kchargingStationDistance);


  private final Command m_driveToTag = new drivetotag(m_drivetrainSubsystem, m_visionSubsystem); 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings

    
    m_chooser.setDefaultOption("Example Trajectory", new TrajectoryRunner(m_drivetrainSubsystem,  m_Trajectory));
    m_chooser.addOption("First Path", new TrajectoryRunner(m_drivetrainSubsystem, firstPath.relativeTo(m_drivetrainSubsystem.getPose())));
    m_chooser.addOption("Newish Path", new TrajectoryRunner(m_drivetrainSubsystem, newishPath.relativeTo(m_drivetrainSubsystem.getPose())));
    m_chooser.addOption("Newerish Path", new TrajectoryRunner(m_drivetrainSubsystem, newerishPath.relativeTo(m_drivetrainSubsystem.getPose())));
    // m_chooser.addOption("Move Forward", m_moveForward);
      SmartDashboard.putData(m_chooser);
    configureBindings();
      
  }

  public void drivetrainDefaultCommand(){
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(-m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrainSubsystem));
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

    //Spins Motor if April Tags are Recognized for 20 Ticks
    m_driverController.a().
        onTrue(new targetFinding(m_drivetrainSubsystem, m_visionSubsystem));
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.b().whileTrue(new InstantCommand(()-> m_drivetrainSubsystem.resetPosition()));

    // m_driverController.a().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "a")) );
    // m_driverController.b().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "b")) );

    // m_driverController.x().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "x")) );

    // m_driverController.y().whileTrue(new RunCommand(() ->SmartDashboard.putString("button pressed", "y")) );

    // //D-Pad buttons
    // m_driverController.povUp().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Up")) );

    // m_driverController.povDown().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Down")) );
    
    // m_driverController.povRight().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Right")) );

    // m_driverController.povLeft().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "Left")) );


    // m_driverController.leftBumper().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "leftBumper")) );

    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "rightBumper")) );

    // m_driverController.leftTrigger().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "leftTrigger")) );

    // m_driverController.rightTrigger().whileTrue(new RunCommand(() -> SmartDashboard.putString("button pressed", "rightTrigger")) );

    // m_driverController.start().whileTrue(new RunCommand (() -> SmartDashboard.putString("button pressed", "startButton")) );


    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new targetFinding(m_drivetrainSubsystem, m_visionSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new drivetotag(m_drivetrainSubsystem, m_visionSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new locateCube(m_drivetrainSubsystem, m_visionSubsystem));
  }

public static double getLeftJoystickY(){
  return m_driverController.getLeftY();
}

public static double getRightJoystickX(){
  return m_driverController.getRightX();
}



public static DrivetrainSubsystem getDrive(){
  return m_drivetrainSubsystem;
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
