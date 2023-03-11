// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.targetFinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.colorSensor;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.subsystems.ledsSubsystem;
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
  private final ledsSubsystem m_LedsSubsystem = new ledsSubsystem();

  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  private final colorSensor m_colorSensorSubsystem = new colorSensor();

  
  // PathPlannerTrajectory firstPath = PathPlanner.loadPath("firstPath", new PathConstraints(4, 3));
  public static PathPlannerTrajectory firstPath = PathPlanner.loadPath("firstPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("Newish path", new PathConstraints(2, 0.8));
  
  PathPlannerTrajectory getOntoChargingStation = PathPlanner.loadPath("GetOntoCSJanky", new PathConstraints(2, 0.8));

  Trajectory m_Trajectory = 
  TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  public RobotContainer() {
    // Configure the trigger bindings
    
    m_chooser.setDefaultOption("Example_Trajectory", (new TrajectoryRunner(m_drivetrainSubsystem, m_LedsSubsystem, m_Trajectory, true)).alongWith(new RunCommand(() -> m_LedsSubsystem.LEDPatronNormal())));
    m_chooser.addOption("First Path", new TrajectoryRunner(m_drivetrainSubsystem, m_LedsSubsystem, firstPath.relativeTo(m_drivetrainSubsystem.getPose()), true).alongWith(new RunCommand(() -> m_LedsSubsystem.LEDPrimerPatron())));
    m_chooser.addOption("Newish Path", new TrajectoryRunner(m_drivetrainSubsystem, m_LedsSubsystem, newishPath.relativeTo(m_drivetrainSubsystem.getPose()), true).alongWith(new RunCommand(() -> m_LedsSubsystem.LEDNewishPath())));
    m_chooser.addOption("Get Onto Charging Station", new TrajectoryRunner(m_drivetrainSubsystem, m_LedsSubsystem, getOntoChargingStation.relativeTo(m_drivetrainSubsystem.getPose()), true).alongWith(new RunCommand(() -> m_LedsSubsystem.LEDGetOntoChargingStation())));
    
    
    SmartDashboard.putData("m_chooser", m_chooser);
  
    // m_chooser.addOption("Move Forward", m_moveForward);`
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
    
    // SmartDashboard.putString("SelectedChooser", m_chooser.getSelected().getName());
   // m_driverController.rightBumper().onTrue(new InstantCommand( () -> m_LedsSubsystem.setLeds(01)));

   m_driverController.leftBumper().onTrue(new InstantCommand( () -> m_LedsSubsystem.SetLedsOff()));//off LED's
   
   m_driverController.start().onTrue(new InstantCommand( () -> m_LedsSubsystem.NeedACube()));//violet
   
   m_driverController.rightBumper().onTrue (new InstantCommand( () -> m_LedsSubsystem.setLeds(0.69)));//yellow

    //Spins Motor if April Tags are Recognized for 20 Ticks
    m_driverController.a().
        onTrue(new targetFinding(m_drivetrainSubsystem, m_visionSubsystem));

    m_driverController.b().whileTrue(new InstantCommand(()-> m_drivetrainSubsystem.resetPosition()));
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
