package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PoseSubsystem extends SubsystemBase {
    private static final PoseSubsystem instance = getInstance();
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;
    private Pose3d camConstant =
            new Pose3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(6.0), new Rotation3d());

    public void initialize(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,  PhotonCamera photonCamera) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.photonCamera = photonCamera;

        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                photonCamera, new Transform3d());
    }

    @Override
    public void periodic() {
        super.periodic();

        double c = Units.inchesToMeters(21.5);
        double camX = c * Math.cos(Math.toRadians(shooterSubsystem.getShooterPitch())) + camConstant.getX();
        double camZ = c * Math.sin(Math.toRadians(shooterSubsystem.getShooterPitch())) + camConstant.getZ();
        Transform3d robotToCam = new Transform3d(camX, camConstant.getY(), camZ, new Rotation3d());

        photonPoseEstimator.setRobotToCameraTransform(robotToCam);

        var tagUpdate = photonPoseEstimator.update();
        if(tagUpdate.isPresent()) {
            var est = tagUpdate.get();
            swerveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        }
    }

    public static PoseSubsystem getInstance() {
        if(instance == null)
            return new PoseSubsystem();
        return instance;
    }
}