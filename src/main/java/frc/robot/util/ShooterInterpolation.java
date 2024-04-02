package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Utility class to perform interpolation to determine the
 * optimal shooter angle given the XY position of the robot on the field.
 *
 * @author Christian
 */
public class ShooterInterpolation {
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map;
    private Pose2d blueSpeaker = new Pose2d(0.5, 5.5, new Rotation2d());
    private Pose2d redSpeaker = new Pose2d(16.0, 5.5, new Rotation2d());
    private final boolean red;

    public ShooterInterpolation(boolean red) {
        this.red = red;
        map = new InterpolatingTreeMap<>();
        map.put(new InterpolatingDouble(4.6 + Units.inchesToMeters(14.0)), new InterpolatingDouble(305.5 - 1.0));
        map.put(new InterpolatingDouble(3.4 + Units.inchesToMeters(14.0)), new InterpolatingDouble(306.5 - 1.0));
        map.put(new InterpolatingDouble(2.6 + Units.inchesToMeters(14.0)), new InterpolatingDouble(309.5 - 2.0));
        map.put(new InterpolatingDouble(2.0 + Units.inchesToMeters(14.0)), new InterpolatingDouble(311.5 - 3.0));
        map.put(new InterpolatingDouble(1.5 + Units.inchesToMeters(14.0)), new InterpolatingDouble(318.5 - 3.0));
        map.put(new InterpolatingDouble(1.0 + Units.inchesToMeters(14.0)), new InterpolatingDouble(328.0 - 3.5));
    }

    @AutoLogOutput
    public double calculate(Pose2d pose) {
        Pose3d speaker = new Pose3d(red ? redSpeaker : blueSpeaker);
        // Get shooter pivot location relative to the center of the robot.
        Transform3d shooterPivot = new Transform3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(6.0), new Rotation3d());
        // Convert robot pose to shooter pivot pose relative to the field.
        Pose3d deltaPose = new Pose3d(pose).plus(shooterPivot);
        // Delta X and Delta Y variables to calculate distance from the speaker.
        double deltaX = deltaPose.getX() - speaker.getX(), deltaY = deltaPose.getY() - speaker.getY();
        // Calculate hypotenuse to get the distance from the speaker & the height of the speaker relative to the pivot.
        double b = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        return map.getInterpolated(new InterpolatingDouble(b)).value;
    }
}