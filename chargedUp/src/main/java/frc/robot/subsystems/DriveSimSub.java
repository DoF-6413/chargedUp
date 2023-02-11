// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSimSub extends SubsystemBase {
  /** Creates a new SimulationSubsystem. */
  
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  
  public DriveSimSub() {
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            // todo: will fix :)
              DCMotor.getNEO(3),
              DrivetrainConstants.kgearing,
              DrivetrainConstants.kMOI,
              DrivetrainConstants.kMass,
              DrivetrainConstants.kwheelRadiusMeters,
              DrivetrainConstants.ktrackWidth,
              // The standard deviations for measurement noise:
              // x and y:          0.001 m
              // heading:          0.001 rad
              // l and r velocity: 0.1   m/s
              // l and r position: 0.005 m
              VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

              private double appliedVoltsLeft = 0.0;
              private double appliedVoltsRight = 0.0;
              
  }

}
