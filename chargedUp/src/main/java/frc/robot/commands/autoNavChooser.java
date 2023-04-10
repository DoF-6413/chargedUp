// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoNavChooser{
  /** Creates a new autoNavChooser. */
  private int m_grid;
  private int m_col;
  Trajectory chosenTraj;
  TrajectoryConfig config;
  public autoNavChooser(int grid, int col) {

    m_grid = grid;
    m_col = col;

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            
        DrivetrainConstants.ksVolts, 
        DrivetrainConstants.kvVoltSecondPerMeter,
        DrivetrainConstants.kaVoltsSecondsSquaredPerMeter), 
        DrivetrainConstants.kinematics, 
        10);
    
    config =
        new TrajectoryConfig(
                0.8,
                0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint).setReversed(true);
  }
    
public Trajectory choosenTrajectory(int grid, int col){
  m_grid = grid;
  m_col = col;
  chosenTraj = new Trajectory();
  

    if (m_grid == 0) {
      if (m_col == 0){
       //"trajectory that gets us the left most grid left colum";
       chosenTraj =  
       TrajectoryGenerator.generateTrajectory(
         new Pose2d(new Translation2d(13.04,0.63),Rotation2d.fromDegrees(180)),
         //go to next to the wood that is on the ground on the left side of the left side of the fild
          List.of(),
          //dont go to any place 
          new Pose2d(new Translation2d(14.66,0.45),Rotation2d.fromDegrees(180)),
         // line up in the left most grid left colum
           config);


      }else if (m_col == 1){
       // "trajectory that gets us the left most grid midle colum";
       chosenTraj = 
       TrajectoryGenerator.generateTrajectory(
         new Pose2d(new Translation2d(13.04,0.63),Rotation2d.fromDegrees(180)),
         //go to next to the wood that is on the ground on  the left side of the fild
          List.of(),
          //dont go to any place 
          new Pose2d(new Translation2d(14.66,1.05),Rotation2d.fromDegrees(180)),
       // line up in the left most grid midle colum
           config);


      }else if (m_col == 2){
        //"trajectory that gets us the left most grid rigth colum";
        chosenTraj = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(13.54,0.63),Rotation2d.fromDegrees(180)),
          //go to next to the wood that is on the ground on the left side of the left side of the fild
           List.of(),
           //dont go to any place 
           new Pose2d(new Translation2d(14.63,1.62),Rotation2d.fromDegrees(180)),
           // line up in the left most grid rigth colum
            config);


      }
      // in this code block we put all the logic for returning trajectories to the
      // left grid

      
    } else if (m_grid == 1) {
      if (m_col == 0 ) {
       //"trajectory that gets us the midle most grid left colum";
       chosenTraj = 
       TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(12.64,2.80),Rotation2d.fromDegrees(180)),
          //go next to the charge station 
           List.of(),
           //dont go to any place 
           new Pose2d(new Translation2d(14.60,2.16),Rotation2d.fromDegrees(180)),
           // line up in the midle most grid left colum
            config);

      } else if (m_col == 1){
       //"trajectory that gets us the midle most grid middle colum";
       TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(12.64,2.80),Rotation2d.fromDegrees(180)),
        //go next to the charge station 
         List.of(),
         //dont go to any place 
         new Pose2d(new Translation2d(14.60,2.74),Rotation2d.fromDegrees(180)),
         // line up in the midle most grid midle colum
          config);

      } else if (m_col == 2){
       //"trajectory that gets us the midle most grid rigth colum";
       chosenTraj = 
       TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(12.64,2.80),Rotation2d.fromDegrees(180)),
        //go next to the charge station 
         List.of(),
         //dont go to any place 
         new Pose2d(new Translation2d(14.60,3.30),Rotation2d.fromDegrees(180)),
         // line up in the midle most grid rigth colum
          config);
        
      }
      // in this code block we put all the logic for returning trajectories to the
      // middle grid
    } else if (m_grid == 2) {
      // in this code block we put all the logic for returning trajectories to the
      // right grid
      if (m_col == 0) {
        // "trajectory that gets us to the rightmost grid left column";
        chosenTraj = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(12.30,4.60),Rotation2d.fromDegrees(180)),
        //go to the rigth side of the charge station 
         List.of(),
         //dont go to any place 
         new Pose2d(new Translation2d(14.60,3.86),Rotation2d.fromDegrees(180)),
         // line up in the rigth most grid left colum
          config);
        
      } else if (m_col == 1) {
      //"trajectory that gets us to the rightmost grid middle column";
      chosenTraj = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(12.30,4.60),Rotation2d.fromDegrees(180)),
        //go to the rigth side of the charge station 
         List.of(),
         //dont go to any place 
         new Pose2d(new Translation2d(14.60,4.40),Rotation2d.fromDegrees(180)),
         // line up in the rigth most grid left colum
          config);
      
      } else if (m_col == 2) {
       // "trajectory that gets us to the rightmost grid right column";
        chosenTraj = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(12.30,4.60),Rotation2d.fromDegrees(180)),
          //go to the rigth side of the charge station 
           List.of(),
           //dont go to any place 
           new Pose2d(new Translation2d(14.60,4.98),Rotation2d.fromDegrees(180)),
           // line up in the rigth most grid left colum
            config);
      }

      return chosenTraj;
    }

    return chosenTraj;
    
  }
}
