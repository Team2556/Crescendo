// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */
  private final static PhotonSubsystem instance = PhotonSubsystem.getInstance();

  private final PhotonCamera camera = new PhotonCamera("photonVision");
double targetX;
double targetY;
double targetRotation;
public static double shootAngle;
double xSquared;
double speakerHeight = Units.inchesToMeters(78);
double speakerHypt;
  public PhotonSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets){
      SmartDashboard.putBoolean("hasTargets", hasTargets);
      PhotonTrackedTarget target = result.getBestTarget();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      targetX = bestCameraToTarget.getX(); 
      targetY = bestCameraToTarget.getY(); 
      targetRotation = bestCameraToTarget.getRotation().getAngle(); 


      xSquared = (targetX * targetX);
      speakerHeight = ( speakerHeight * speakerHeight);
      speakerHypt = Math.sqrt(xSquared + speakerHeight);
      shootAngle = Math.atan(speakerHeight / targetX);


  
 } else {
  SmartDashboard.putBoolean("hasTargets", hasTargets);
 }
    
  }
  public static PhotonSubsystem getInstance() {
    if (instance == null) {
      return new PhotonSubsystem();
    }
    return instance;
  } 
}
