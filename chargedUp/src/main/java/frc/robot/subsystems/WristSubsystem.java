// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final CANSparkMax m_wrist;
  private final RelativeEncoder m_wristEncoder;
  public WristSubsystem() {
    m_wrist = new CANSparkMax(10, MotorType.kBrushless);
    m_wristEncoder = m_wrist.getEncoder();
    m_wrist.setSmartCurrentLimit(5);
    m_wrist.setIdleMode(IdleMode.kBrake);
  }

    public final void spinWrist(double speed){
      m_wrist.set(speed);
    }

    public final void stopWrist(){
      m_wrist.set(0);
    }

    public final void resetWrist(){
     m_wristEncoder.setPosition(0);
    }

    public final double getPosition(){
      return m_wristEncoder.getPosition();
    }

    public final void setCurrentLimit(int currentLimit){
      m_wrist.setSmartCurrentLimit(currentLimit);
    }

    public final double getOutputCurrent(){
      return m_wrist.getOutputCurrent();
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Position", getPosition());
    SmartDashboard.putNumber("Current", getOutputCurrent());
  }
}
