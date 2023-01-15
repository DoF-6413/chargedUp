// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    PortForwarder.add(5800, "photonvision.local", 5800);
  }
  
  //todo: provide portforwaring to connect without radio
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private PhotonPipelineResult results; 
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getLatestResult();
    SmartDashboard.putBoolean("target?", seeTarget());
    SmartDashboard.putNumber("Best fiducial", getBestFiducial());
  }

  public boolean seeTarget(){

    return results.hasTargets();
  }

  //This returns ALL tags/targets the camera identifies
  // public List<PhotonTrackedTarget> identifyTags(){
  //   return results.getTargets();
  // }

  public int getBestFiducial(){
    return (this.seeTarget() == true) ? results.getBestTarget().getFiducialId() : 0;
  }
}
