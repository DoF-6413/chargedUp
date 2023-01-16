// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final double CAMERA_HEIGHT_METERS = 1;
  private static final double CAMERA_PITCH_RADIANS = 0;
  private static final double TARGET_HEIGHT_METERS = 1;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    PortForwarder.add(5800, "photonvision.local", 5800);
  }
  
  //todo: provide portforwaring to connect without radio
  PhotonCamera camera = new PhotonCamera("Logi_Webcam_C920e");
  
  private PhotonPipelineResult results; 
  public PhotonTrackedTarget target;
  public Double yaw;
  public Double pitch;
  public Transform3d camToTarget;
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getLatestResult();
    SmartDashboard.putBoolean("target?", seeTarget());
    SmartDashboard.putNumber("Best fiducial", getBestFiducial());
   // SmartDashboard.putNumber("distance X", distanceFinder().getX());
    // SmartDashboard.putNumber("distance Y", distanceFinder().getY());
    // SmartDashboard.putNumber("distance Z", distanceFinder().getZ());
   
  }

  public boolean seeTarget(){
    if( results.hasTargets() == true){
      target = results.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      camToTarget = target.getBestCameraToTarget();
      SmartDashboard.putNumber("distance X", camToTarget.getX());
    SmartDashboard.putNumber("distance Y", camToTarget.getY());
    SmartDashboard.putNumber("distance Z", camToTarget.getZ());
    }
    return results.hasTargets();
  }

  public int getBestFiducial(){
    return (this.seeTarget() == true) ? results.getBestTarget().getFiducialId() : 0;
  }


  
  public Transform3d distanceFinder(){
Transform3d m_default = new Transform3d();
return seeTarget() == true ?  results.getBestTarget().getBestCameraToTarget() : m_default ; 
  }

  public boolean isHuge() {
    boolean grande =  (this.seeTarget() == true) ? results.getBestTarget().getArea() > .5 : false;
    boolean huge =  (this.seeTarget() == true) ? grande: false;
    return huge;
  }

}
