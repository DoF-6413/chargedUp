// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SimulationDevices.NavXWrapper;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  AHRS m_gyro;
  public GyroSubsystem() {
    m_gyro = new NavXWrapper(SPI.Port.kMXP, (byte) 200) ;
  }

  public double getAngle(){
    return m_gyro.getAngle();
  }

  public void resetYaw(){
    m_gyro.zeroYaw();
  }

  public Rotation2d getRotation2d(){
  return  m_gyro.getRotation2d() != null ? m_gyro.getRotation2d() : null ;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getGyroAngle(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360)*(-1);
  }
}
