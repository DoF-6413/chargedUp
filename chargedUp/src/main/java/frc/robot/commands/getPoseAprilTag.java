// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.time.chrono.ThaiBuddhistChronology;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem; 
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.GyroSubsystem;



public class getPoseAprilTag extends CommandBase {
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
private final VisionSubsystem m_VisionSubsystem;
private final GyroSubsystem m_GyroSubsystem;
public static Pose2d m_derivedPose;
public double distanceTo;
  /** Creates a new getPoseAprilTag. */
  public getPoseAprilTag(DrivetrainSubsystem drive, VisionSubsystem vision, GyroSubsystem gyro) {
    m_DrivetrainSubsystem = drive;
    m_VisionSubsystem = vision;
    m_GyroSubsystem = gyro;
    m_derivedPose = null;
    
   addRequirements(m_DrivetrainSubsystem, m_VisionSubsystem, m_GyroSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    //COMMENTED OUT because this approach did not work and we couldn't get the right yaw 
    // grab apriltag -20 = camera angle
   }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_VisionSubsystem.getBestFiducial() != null){
      //transform radian to degress
    double thetaRadian = ((m_VisionSubsystem.getBestFiducial().getBestCameraToTarget().getRotation().getZ()*180)/3.14159265359);
    double theta = (thetaRadian *180)/3.14159265359;
    if (theta > 0){
theta = 180 - theta;
    }if (theta < 0){
      theta += 180;
    }
    System.out.println(theta); 
        
    distanceTo = m_VisionSubsystem.getBestFiducial().getBestCameraToTarget().getX();
     
        SmartDashboard.putNumber("distance To April Tag", distanceTo);
        
        int currentTag = m_VisionSubsystem.getBestFiducialID();
        SmartDashboard.putNumber("Current Tag", currentTag);
    
        Pose2d targetPose = VisionConstants.tagPoses[currentTag];
        if(targetPose != null){
          
          SmartDashboard.putNumber("target Pose x",targetPose.getX());
          SmartDashboard.putNumber("target Pose Y",targetPose.getY());
          

        double xOffset = distanceTo * new Rotation2d(thetaRadian).getCos();
        SmartDashboard.putNumber("X Offset", xOffset);
    
        double yOffset = distanceTo * new Rotation2d(thetaRadian).getSin();
        SmartDashboard.putNumber("Y Offset", yOffset);
    
        double derivedX = targetPose.getX() + xOffset;
        SmartDashboard.putNumber("derived X", derivedX);
    
        double derivedY = targetPose.getY() + yOffset;
        SmartDashboard.putNumber("derived Y", derivedY);
        
        m_derivedPose = new Pose2d(derivedX, derivedY, m_GyroSubsystem.getRotation2d() );
        SmartDashboard.putNumber("derived Pose X", m_derivedPose.getX());
        SmartDashboard.putNumber("derived Pose Y",targetPose.getY());

      
    // distanceTo = PhotonUtils.calculateDistanceToTargetMeters(
    //   1.13, 1.27, 0, Units.radiansToDegrees(m_VisionSubsystem.getBestFiducial().getPitch()));
     
    // System.out.println( m_derivedPose.toString());
  m_DrivetrainSubsystem.resetOdometry(m_derivedPose);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
