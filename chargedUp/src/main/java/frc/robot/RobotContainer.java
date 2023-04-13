// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TelescoperConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmPIDSubsystem;
// import frc.robot.commands.targetFinding;
// import frc.robot.commands.ArmPID;
// import frc.robot.commands.targetFinding;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
// import frc.robot.commands.targetFinding;
// import frc.robot.commands.ArmPID;
// import frc.robot.commands.targetFinding;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.ArmPID;
import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.colorSensor;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
// import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.commands.ArmControls.TelescoperReset;
import frc.robot.commands.Autos.CenterLScoreOutBalance;
import frc.robot.commands.Autos.ScoreBalance;
import frc.robot.commands.Autos.ScoreGetScore;
import frc.robot.commands.Autos.ScoreHigh;
import frc.robot.commands.Autos.ScoreMovePickupScore;
import frc.robot.commands.Autos.scoreRun;
import frc.robot.commands.TeleopAutomations.BackIn;
import frc.robot.commands.TeleopAutomations.ConePickUp;
import frc.robot.commands.TeleopAutomations.PickupCone;
import frc.robot.commands.TeleopAutomations.PlaceHigh;
import frc.robot.commands.TeleopAutomations.PlaceMid;
import frc.robot.commands.TeleopAutomations.PositionHigh;
import frc.robot.commands.TeleopAutomations.PositionMid;
import frc.robot.commands.TeleopAutomations.PositionPickUp;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.TrajectoryRunner;
import frc.robot.commands.getEstimatedPose;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.TelescoperSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;

// import frc.robot.subsystems.colorSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();


  public final ArmPIDSubsystem m_ArmPIDSubsystem = new ArmPIDSubsystem();

  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_gyroSubsystem );
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private final TelescoperSubsystem m_telescoperSubsystem = new TelescoperSubsystem();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  
  
  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  // private final colorSensor m_colorSensorSubsystem = new colorSensor();
  
  // PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final PoseEstimator m_PoseEstimatorSubsystem = new PoseEstimator(m_gyroSubsystem, m_drivetrainSubsystem, m_visionSubsystem, DrivetrainConstants.kinematics );
  
  // private Trajectory m_chosenTraj;
  private autoNavChooser m_AutoNavChooser = new autoNavChooser(0, 0);
  private int grid, col;

  //warning means not used, but its here so it calls the periodic for the subsystem DO NOT REMOVE
  // private final colorSensor m_colorSensorSubsystem = new colorSensor();

  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(2, 0.8));
  PathPlannerTrajectory newishPath = PathPlanner.loadPath("VisionTest", new PathConstraints(2, 0.8));
  PathPlannerTrajectory getOntoChargingStation = PathPlanner.loadPath("GetOntoCSJanky", new PathConstraints(2, 0.8));
 PathPlannerTrajectory RightRed2 = PathPlanner.loadPath("RightRed2", new PathConstraints(4, 4));
 PathPlannerTrajectory runOutCommunity = PathPlanner.loadPath("OutCommunity", new PathConstraints(1.5, 0.8));
 PathPlannerTrajectory overCSBalance = PathPlanner.loadPath("OverCSBalance", new PathConstraints(1.5, 1.2));
 

 
 PathPlannerTrajectory RightRed2Traj = PathPlanner.generatePath(
  new PathConstraints(0.5,0.5), 
  new PathPoint(m_PoseEstimatorSubsystem.getcurrentPose().getTranslation(),m_gyroSubsystem.getRotation2d()),
new PathPoint(RightRed2.getInitialPose().getTranslation(),RightRed2.getInitialPose().getRotation())
);

  PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(0.2, 0.5), 
    new PathPoint(new Translation2d(14.5, 7.32), new Rotation2d(0)), // position, he
   new PathPoint(new Translation2d(15.62,7.32),new Rotation2d(-0.35))
    );

    // Trajectory traj12092007 = TrajectoryGenerator.generateTrajectory(
    //   m_PoseEstimatorSubsystem.getcurrentPose(),
    //   List.of(new Translation2d(15.57, 7.32)),
    //     new Pose2d(new Translation2d(15.57,7.32),new Rotation2d(3.14)),
    //       new TrajectoryConfig(Units.feetToMeters(1.0), Units.feetToMeters(1.0)));
    


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public final static CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);

      public final GenericHID m_buttonBoard = new GenericHID(OperatorConstants.kJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    // Configure the trigger bindings
    // m_chooser.setDefaultOption("Test Path", new TrajectoryRunner(m_drivetrainSubsystem, testPath.relativeTo(m_drivetrainSubsystem.getPose()), true));
    m_chooser.addOption("Score Grid1 TL Run", new scoreRun(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_PoseEstimatorSubsystem));
    // m_chooser.addOption("Score Grid1 TR Around CS Balance", new G1TRAroundCSBalance(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    m_chooser.addOption("Score Grid2 TL Over CS Balance", new CenterLScoreOutBalance(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem, m_PoseEstimatorSubsystem));
    // m_chooser.addOption("Score Grid2 TR Over CS Balance", new CenterRScoreOutBalance(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    // m_chooser.addOption("Score Grid3 TL Around CS Balance", new G3TLAroundCSBalance(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem));
    // m_chooser.addOption("Score Grid3 TR Run", new ScoreRunRight(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem));
    // m_chooser.addOption("Score Grid1 TL Run Follow Path With Events", new ScoreRunFollowWithEvents(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_drivetrainSubsystem));
    m_chooser.addOption("Out of Community", new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, ()-> runOutCommunity.relativeTo(m_PoseEstimatorSubsystem.getcurrentPose()), true));
    m_chooser.addOption("Out of Community and Balance", new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem,()-> overCSBalance.relativeTo(m_PoseEstimatorSubsystem.getcurrentPose()), true));
    m_chooser.addOption("Score High", new ScoreHigh(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_drivetrainSubsystem));
    m_chooser.addOption("Score and Balance", new ScoreBalance(m_ArmPIDSubsystem, m_drivetrainSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_gyroSubsystem, m_PoseEstimatorSubsystem));
    m_chooser.addOption("Score Pickup Score", new ScoreMovePickupScore(m_drivetrainSubsystem, m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_PoseEstimatorSubsystem));
    m_chooser.addOption("Score Pickup Score Optimized", new ScoreGetScore(m_drivetrainSubsystem, m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_PoseEstimatorSubsystem));

    SmartDashboard.putData("hahah", m_chooser);
    m_chooser.setDefaultOption("Test Path", new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem,()-> testPath.relativeTo(m_PoseEstimatorSubsystem.getcurrentPose()), true));
    m_chooser.addOption("Newish Path", new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, ()->newishPath.relativeTo(m_PoseEstimatorSubsystem.getcurrentPose()), true));
    
      SmartDashboard.putData(m_chooser);
    configureBindings();
    defaultCommands();
      
  }

  public void defaultCommands(){
    m_drivetrainSubsystem.setDefaultCommand(new RunCommand(() ->
     m_drivetrainSubsystem.setRaw(-m_driverController.getLeftY(), -m_driverController.getRightX()*0.75), m_drivetrainSubsystem));

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
        // m_auxController.start().onTrue(new RotationReset(m_ArmPIDSubsystem));
        m_auxController.start().onTrue(new InstantCommand(()-> m_ArmPIDSubsystem.resetRotationPosition()));
        //This runs Endeffector to Collect Cone
        // m_auxController.a().
        // onTrue(new InstantCommand(()-> m_ArmPIDSubsystem.resetRotationPosition()));
        m_auxController.leftTrigger().
        onTrue(new PickupCone(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem)).
        onFalse (
          // new ConditionalCommand(
          // new CubePIckUp(m_wristSubsystem, m_telescoperSubsystem, m_ArmPIDSubsystem));
          new ConePickUp(m_wristSubsystem, m_telescoperSubsystem, m_ArmPIDSubsystem));
          // ()-> m_colorSensorSubsystem.getColor() == m_colorSensorSubsystem.kpurple));
        
        // m_auxController.rightTrigger().onTrue(
        //   new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(-0.3))).
        //   onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));

        m_auxController.y()
            .onTrue(
              // new TelescoperPID(m_telescoperSubsystem, TelescoperConstants.kMaxExtention))
              new PositionHigh(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem, m_wristSubsystem))
            .onFalse(
              new PlaceHigh(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem));

      

        // m_auxController.x()
        //     .onTrue(
        //       new PositionMid(m_telescoperSubsystem, m_ArmPIDSubsystem, m_endEffectorSubsystem, m_wristSubsystem))
        //       .onFalse(
        //         new PlaceMid(m_ArmPIDSubsystem, m_telescoperSubsystem, m_endEffectorSubsystem));

        // m_auxController.a().onTrue(
        //   new TelescoperPID(m_telescoperSubsystem, 50)).
        //   onFalse(new TelescoperPID(m_telescoperSubsystem, 0));

        //   m_auxController.b().onTrue(
        //     new TelescoperPID(m_telescoperSubsystem, 16)).
        //     onFalse(new TelescoperPID(m_telescoperSubsystem, 0));

        // m_auxController.leftBumper().onTrue(new InstantCommand(()-> m_wristSubsystem.spinWrist(.50)))
        // .onFalse(new InstantCommand(()-> m_wristSubsystem.stopWrist()));

        // m_auxController.rightBumper().onTrue(new InstantCommand(()-> m_wristSubsystem.spinWrist(-.50)))
        // .onFalse(new InstantCommand(()-> m_wristSubsystem.stopWrist()));


    // m_driverController.rightTrigger().onTrue(new InstantCommand(()-> m_LEDSubsystem.NeedACube()));
    // m_driverController.leftTrigger().onTrue(new InstantCommand(()-> m_LEDSubsystem.NeedACone()));

    // m_driverController.leftBumper().onTrue(new PositionPickUp(m_telescoperSubsystem, m_ArmPIDSubsystem, m_endEffectorSubsystem))
    // .onFalse(new BackIn(m_telescoperSubsystem, m_ArmPIDSubsystem));

    // m_driverController.rightBumper().onTrue(
    //   new InstantCommand(()-> m_endEffectorSubsystem.spinEndEffector(0.5))).
    //   onFalse(new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector()));

      //if the driver controller 
      // if(m_driverController.a().getAsBoolean() == true){
      //   grid = 0;

      // } if (m_driverController.b().getAsBoolean() == true){
      //   col = 0;
      // }


    m_driverController.y().whileTrue(new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, m_AutoNavChooser.choosenTrajectory(), false));

      // m_driverController.a().onTrue( new getEstimatedPose(m_gyroSubsystem, m_drivetrainSubsystem, m_visionSubsystem, m_PoseEstimatorSubsystem));

    //   m_driverController.y().whileTrue(
    // trajGenCommand());

      new JoystickButton(m_buttonBoard, 1).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(0)));
      
      new JoystickButton(m_buttonBoard, 2).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(1)));
      
      new JoystickButton(m_buttonBoard, 3).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(2)));
      
      new JoystickButton(m_buttonBoard, 4).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(0)));
      
      new JoystickButton(m_buttonBoard, 5).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(1)));
      
      new JoystickButton(m_buttonBoard, 6).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(2)));
      
      new JoystickButton(m_buttonBoard, 7).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(0)));
      
      new JoystickButton(m_buttonBoard, 8).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(1)));
      
      new JoystickButton(m_buttonBoard, 9).onTrue(new InstantCommand(()-> m_AutoNavChooser.setCol(2)));

      //grid setters
      
      new JoystickButton(m_buttonBoard, 10).onTrue(new InstantCommand(()-> m_AutoNavChooser.setGrid(0)));
      
      new JoystickButton(m_buttonBoard, 11).onTrue(new InstantCommand(()-> m_AutoNavChooser.setGrid(1)));
      
      new JoystickButton(m_buttonBoard, 12).onTrue(new InstantCommand(()-> m_AutoNavChooser.setGrid(2)));

      //else{new InstantCommand(()-> m_endEffectorSubsystem.stopEndEffector());}
      // m_driverController.y().whileTrue(
      //   // new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem,  traj1, false));
      //   new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, traj12092007.relativeTo(m_PoseEstimatorSubsystem.getcurrentPose()), false));
  }


  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void disablePIDSubsystems() {
    m_ArmPIDSubsystem.disable();
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }

  // public void choosePath(){
  //   var autoVoltageConstraint =
  //   new DifferentialDriveVoltageConstraint(
  //       new SimpleMotorFeedforward(
            
  //       DrivetrainConstants.ksVolts, 
  //       DrivetrainConstants.kvVoltSecondPerMeter,
  //       DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
  //       DrivetrainConstants.kinematics, 
  //       10);
  //   TrajectoryConfig config =
  //   new TrajectoryConfig(
  //           0.4,
  //           0.5)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DrivetrainConstants.kinematics)
  //       // Apply the voltage constraint
  //       .addConstraint(autoVoltageConstraint).setReversed(true);
  //   Trajectory exampleTrajectory =
  //   TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(new Translation2d(14.4, 4.5),Rotation2d.fromDegrees(180)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(14.5,4.0)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(new Translation2d(14.6, 3.33),Rotation2d.fromDegrees(180)),
  //       // Pass config
  //       config);
  //   m_chosenTraj = exampleTrajectory;
  // }

  // public Command trajGenCommand() {
  //   // Create a voltage constraint to ensure we don't accelerate too fast
  //   var autoVoltageConstraint =
  //       new DifferentialDriveVoltageConstraint(
  //           new SimpleMotorFeedforward(
                
  //           DrivetrainConstants.ksVolts, 
  //           DrivetrainConstants.kvVoltSecondPerMeter,
  //           DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
  //           DrivetrainConstants.kinematics, 
  //           10);

  //   // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(
  //               0.4,
  //               0.5)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(DrivetrainConstants.kinematics)
  //           // Apply the voltage constraint
  //           .addConstraint(autoVoltageConstraint).setReversed(true);
            
    

  //   // An example trajectory to follow.  All units in meters.
   
  //    Trajectory  leftgridleft = 
  //     TrajectoryGenerator.generateTrajectory(
  //       new Pose2d(new Translation2d(13.04,0.63),Rotation2d.fromDegrees(180)),
  //       //go to next to the wood that is on the ground on the left side of the left side of the fild
  //        List.of(),
  //        //dont go to any place 
  //        new Pose2d(new Translation2d(14.66,0.45),Rotation2d.fromDegrees(180)),
  //        //go to the first grid of the left side in the left side of the fild
  //         config);
  //         //  RamseteCommand ramseteCommand = new RamseteCommand(
  //         //       exampleTrajectory, 
  //         //       m_PoseEstimatorSubsystem::getcurrentPose, 
  //         //     // m_ramseteCommand =
  //         //       new RamseteController(
  //         //         DrivetrainConstants.kRamseteB, 
  //         //         DrivetrainConstants.kRamseteZeta)
  //         //         // ;
  //         //         , 
  //         //       new SimpleMotorFeedforward(
  //         //         DrivetrainConstants.ksVolts, 
  //         //         DrivetrainConstants.kvVoltSecondPerMeter,
  //         //         DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
  //         //         DrivetrainConstants.kinematics, 
  //         //         m_drivetrainSubsystem::getWheelSpeeds, 
  //         //       new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
  //         //       new PIDController(DrivetrainConstants.kMoveP, DrivetrainConstants.kMoveI, DrivetrainConstants.kMoveD), 
  //         //       m_drivetrainSubsystem::tankDrive, 
  //         //       m_drivetrainSubsystem, m_PoseEstimatorSubsystem);

  //   // // Reset odometry to the starting pose of the trajectory.
  //   // m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
  
  //   // Run path following command, then stop at the end.
  //   return new TrajectoryRunner(m_drivetrainSubsystem, m_PoseEstimatorSubsystem, m_chosenTraj, false);
  
//  }
}


