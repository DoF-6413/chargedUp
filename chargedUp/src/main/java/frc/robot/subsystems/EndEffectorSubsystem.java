// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndEffectorSubsystem. */
  
private final TalonFX m_endEffectorMotor;

  public EndEffectorSubsystem() {
    
    m_endEffectorMotor = new TalonFX(EndEffectorConstants.kEndEffectorCANID);
    m_endEffectorMotor.setNeutralMode(NeutralMode.Brake);
    m_endEffectorMotor.setInverted(true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboardCalls();
  }

  public void SmartDashboardCalls(){
    // SmartDashboard.putNumber("EndEffectorPosition", getEndEffectorPosition());
  }

  

  public double getEndEffectorPosition(){
    return m_endEffectorMotor.getSelectedSensorPosition();
  }

  public void resetEndEffectorPosition(){ 
    m_endEffectorMotor.setSelectedSensorPosition(0);
  }

  public void spinEndEffector(double speed){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, speed);
  }


  public void stopEndEffector(){
    m_endEffectorMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

}
