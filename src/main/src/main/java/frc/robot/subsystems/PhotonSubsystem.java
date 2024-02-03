package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class PhotonSubsystem extends SubsystemBase
{
    private static final PhotonSubsystem instance = getInstance();
    //final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(29); //height of Limelight
    
    // Angle between horizontal and the camera.
    //final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); //angle of camera
    //distance between the camera and target
   // final double GOAL_RANGE_METERS = Units.inchesToMeters(16); 
   // double TARGET_HEIGHT_METERS;
   static PhotonCamera camera = new PhotonCamera("photonvision");
    
   static PhotonPipelineResult result;

   static PhotonTrackedTarget target;

    public PIDController xPID = new PIDController(0.1, 0, 0);
    public PIDController yPID = new PIDController(0.1, 0, 0);
    public PIDController zPID = new PIDController(0.1, 0, 0);

    
   public double horizontalDistance() {
    Transform3d pose = target.getBestCameraToTarget();
    double yDistance = pose.getY();
    return yDistance;
   }
   public double verticalDistance() {
    Transform3d pose = target.getBestCameraToTarget();
    double xDistance = pose.getX();
    return xDistance;
   }
   public double zAngle() {
    Transform3d pose = target.getBestCameraToTarget();
    double angle = pose.getZ();
    return angle;
   }
            
            // List<TargetCorner> corners = target.getCorners();
             public double movement;
            
             public double Tag(){ //Calculates needed X-distance between robot and Apriltag
            //     double range = 0; //Linear distance between robot and Apriltag
            //     double movement = 0; //Stores the distance robot needs to move in inches
                SmartDashboard.putNumber("Xdistance", verticalDistance());
                SmartDashboard.putNumber("Ydistance", horizontalDistance());
                SmartDashboard.putNumber("zAngle", zAngle());
                return movement;
            }

            //outputs the tracked id to smart dashboard
                //  private void updateSmartDashboard() {
                    
                // }
                public void periodic() {
                    //updateSmartDashboard();
                    result = camera.getLatestResult();
                    if(result == null)
                    return;
                    target = result.getBestTarget();
                    // if(target == null)
                    // return;
                    SmartDashboard.putBoolean("Target Found", target != null);
                    // SmartDashboard.putNumber("Xdistance", verticalDistance());
                    // SmartDashboard.putNumber("Ydistance", horizontalDistance());
                    // SmartDashboard.putNumber("zAngle", zAngle());
                }
                // public void targetID() {
                // }
                public static PhotonSubsystem getInstance() {
                    if(instance != null)
                        return instance;
                    return new PhotonSubsystem();
                }
            
}