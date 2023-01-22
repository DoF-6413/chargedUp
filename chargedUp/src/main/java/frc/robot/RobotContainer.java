// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.colorSensor;
// import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drivetotag;
import frc.robot.commands.locateCube;
// import frc.robot.commands.targetFinding;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
// import frc.robot.commands.targetFinding;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.colorSensor;
// import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.ArmControls.RotationPID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  private final colorSensor m_colorSensorSubsystem = new colorSensor();

  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("Newish path", new PathConstraints(2, 0.8));
  
  PathPlannerTrajectory getOntoChargingStation = PathPlanner.loadPath("GetOntoCSJanky", new PathConstraints(2, 0.8));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public final static CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings
    m_chooser.setDefaultOption("Test Path", new TrajectoryRunner(m_drivetrainSubsystem, testPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Newish Path", new TrajectoryRunner(m_drivetrainSubsystem, newishPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Get Onto Charging Station", new TrajectoryRunner(m_drivetrainSubsystem, getOntoChargingStation.relativeTo(m_drivetrainSubsystem.getPose()), true));
      SmartDashboard.putData(m_chooser);
    configureBindings();
      
  }

  public void drivetrainDefaultCommand(){
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
        //This extends telescoper 
        m_auxController.start().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(0.8)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        //This unextends telescoper 
        m_auxController.back().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinTelescopingMotor(-0.8)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopTelescopingMotor()));

        //This runs Endeffector to Collect Cube
        m_auxController.b().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.07)));

        //This runs Endeffector to Collect Cone
        m_auxController.a().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(0.5)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopEndEffector()));
        
        
        // This runs Endeffector to eject game peices
        m_auxController.rightTrigger().
        onTrue(new InstantCommand(()-> m_armSubsystem.spinEndEffector(-0.2)))
        .onFalse(new InstantCommand(()-> m_armSubsystem.stopEndEffector()));

        //This runs rotation motors to 77.5 degrees
        m_auxController.y().
        onTrue(new RotationPID(m_armSubsystem, 77.5));
        
        //This runs rotation motors to -77.5 degrees
        m_auxController.x(). onTrue(new RotationPID(m_armSubsystem, -77.5));

        //This resets the encoder value of the arm where it is currently located
        m_auxController.leftBumper().onTrue(new InstantCommand(()-> m_armSubsystem.resetRotationPosition()));
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
