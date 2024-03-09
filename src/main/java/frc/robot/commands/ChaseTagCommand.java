// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.ROBOT_TO_CAMERA;

public class ChaseTagCommand extends Command {

  //static PhotonCamera camera = new PhotonCamera("photonvision");
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(.8, 4);
  
  private static final int TAG_TO_CHASE = 1;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private int targetID;
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator estimator;
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(.1, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(.1, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(.1, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

   

  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    this.photonCamera = photonCamera;
    this.swerveSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    estimator = new PhotonPoseEstimator(Constants.FieldK.kFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, ROBOT_TO_CAMERA);

    xController.setTolerance(0.35);
    yController.setTolerance(0.35);
    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, /*Units.degreesToRadians(355)*/0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();

   
    
    // var tagUpdate = estimator.update();
    // if(tagUpdate.isPresent()) {
    //   var est = tagUpdate.get();
    //   swerveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
    // }

    if (photonRes.hasTargets()) {
        
      PhotonTrackedTarget target = photonRes.getBestTarget();

      Transform3d goalPose = target.getBestCameraToTarget();

      targetID = target.getFiducialId();
      // Find the tag we want to chase
      // var targetOpt = photonRes.getTargets().stream()
      //     .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
      //     .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
      //     .findFirst();
      // if (targetOpt.isPresent()) {
      //   var target = targetOpt.get();
      //   // This is new target data, so recalculate the goal
      //   lastTarget = target;
      //   SmartDashboard.putString("Tag Pose", target.getBestCameraToTarget().toString());
      //   // Transform the robot's pose to find the camera's pose
      //   var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

      //   // Trasnform the camera's pose to the target's pose
      //   var camToTarget = target.getBestCameraToTarget();
      //   var targetPose = cameraPose.transformBy(camToTarget);
        
      //   // Transform the tag's pose to set our goal
      //   var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

      //   SmartDashboard.putString("GOAL POSE", goalPose.toString());
      //   SmartDashboard.putString("CURRENT ROBOT POSE", poseProvider.get().toString());

        // Drive
        xController.setGoal(goalPose.getX()); 
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getAngle());
        SmartDashboard.putNumber("Xvaluetag", goalPose.getX());
        SmartDashboard.putNumber("Yvaluetag", goalPose.getY());
        //omegaController.setGoal(goalPose.getRotation().toPose2d();
      //}
    }
    
    if (lastTarget == null) {
      // No target has been visible
      swerveSubsystem.drive(new ChassisSpeeds());
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }
      else {
        omegaSpeed = 0; 
      }

      if (targetID == 7 || targetID == 4) {
      swerveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        // ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, omegaSpeed, robotPose2d.getRotation()));
      }
    }
  }
 
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds());
  }

}