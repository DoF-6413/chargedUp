// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmMotor;
import frc.robot.commands.ArmControls.RotationPID;
import frc.robot.commands.ArmControls.TelescoperPID;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.colorSensor;


public class ArmSubsystem extends SubsystemBase {
private DifferentialDriveWheelSpeeds m_DifferentialDriveWheelSpeeds;
private TelescoperSubsystem m_TelescoperSubsystem;
private final CANSparkMax m_leftRotationMotor;
private final CANSparkMax m_rightRotationMotor;
private final RelativeEncoder m_RotationEncoder;
private DrivetrainSubsystem drive;
private colorSensor m_ColorSensor;




  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftRotationMotor = new CANSparkMax(ArmMotor.leftRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor = new CANSparkMax(ArmMotor.rightRotationMotor.CAN_ID, MotorType.kBrushless);
    m_rightRotationMotor.follow(m_leftRotationMotor);
    m_leftRotationMotor.setIdleMode(IdleMode.kBrake);
    m_rightRotationMotor.setIdleMode(IdleMode.kBrake);
    m_leftRotationMotor.setSmartCurrentLimit(ArmConstants.kRotationCurrentLimit);
    
    m_RotationEncoder = m_leftRotationMotor.getEncoder();
    m_RotationEncoder.setPositionConversionFactor(ArmConstants.kRotationPositionConversion);
    
    m_rightRotationMotor.follow(m_leftRotationMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboardCalls();
  }
public void armSafety(DrivetrainSubsystem m_drive){

  // if the robot see a game piece and its moving then put the arm down
  if(m_drive.getAverageMotorSpeed() > 0.3 && (( m_ColorSensor.getColor() == colorSensor.kpurple)||(m_ColorSensor.getColor() == colorSensor.kyellow)))
  {new RotationPID(this, 0);}

}


  public Command runDefaults(Double joystick, DrivetrainSubsystem drive){
    return  new ParallelCommandGroup(new RunCommand(() -> spinRotationMotors(joystick)), 
    new RunCommand(() -> armSafety(drive)));
}



  public void SmartDashboardCalls(){
    SmartDashboard.putNumber("RotationPosition", getRotationPosition());    
  }

  public double getRotationPosition(){
    return m_RotationEncoder.getPosition();
  }


  
  public void resetRotationPosition(){
    m_RotationEncoder.setPosition(0);
  }

  public void spinRotationMotors(double speed){
    m_leftRotationMotor.set(speed);
    SmartDashboard.putNumber("Rotation speed", speed);
  }

  public void stopRotationMotors(){
    m_leftRotationMotor.set(0);
  }

  public Boolean isInFramePerimeter(){
    return ((this.getRotationPosition() < 35) && (this.getRotationPosition() > -35 )) ?  true :  false;
  }
  
}
