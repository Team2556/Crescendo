package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PoseSubsystem extends SubsystemBase {
    private static final PoseSubsystem instance = getInstance();
    private SwerveSubsystem swerveSubsystem;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;
    private Transform3d camConstant =
            new Transform3d(Units.inchesToMeters(-13.0), 0.0, Units.inchesToMeters(7.8), new Rotation3d(0.0, Math.toRadians(-31.2), Math.toRadians(180)));

    public void initialize(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera) {
        this.swerveSubsystem = swerveSubsystem;
        this.photonCamera = photonCamera;

        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                photonCamera, camConstant);
    }

    @Override
    public void periodic() {
        super.periodic();

        var tagUpdate = photonPoseEstimator.update();
        if(tagUpdate.isPresent()) {
            var est = tagUpdate.get();
            swerveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        }
    }

    public Pose2d getPose() {
        return swerveSubsystem.getPose();
    }

    public double getX() {
        return getPose().getX();
    }

    public double getY() {
        return getPose().getY();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public static PoseSubsystem getInstance() {
        if(instance == null)
            return new PoseSubsystem();
        return instance;
    }
}