// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.AutoScore2;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivetotag;
import frc.robot.commands.targetFinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonActionListener;
import edu.wpi.first.wpilibj.XboxController.Button;

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
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();


  // list of autos
  private final Command m_autoScore = new AutoScore2();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings

    
    m_chooser.setDefaultOption("Auto Score", m_autoScore);
    m_chooser.addOption("Auto Score", m_autoScore);
      SmartDashboard.putData(m_chooser);
      
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrainSubsystem));
    configureBindings();


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
    new JoystickButton(m_driverController, XboxController.Button.kA.value).
        onTrue(new targetFinding(m_drivetrainSubsystem, m_visionSubsystem));
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

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
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new drivetotag(m_drivetrainSubsystem, m_visionSubsystem, 1.0));
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
