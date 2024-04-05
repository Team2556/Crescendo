package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class to perform interpolation to determine the
 * optimal shooter angle given the XY position of the robot on the field.
 *
 * @author Christian
 */
public class ShooterInterpolation {
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map;
    private static Pose2d blueSpeaker = new Pose2d(0.05, 5.5, new Rotation2d());
    private static Pose2d redSpeaker = new Pose2d(16.5, 5.5, new Rotation2d());
    private static boolean red;

    public ShooterInterpolation(boolean red) {
        ShooterInterpolation.red = red;
        map = new InterpolatingTreeMap<>();
        map.put(new InterpolatingDouble(4.6 + Units.inchesToMeters(14.0)), new InterpolatingDouble(305.5));
        map.put(new InterpolatingDouble(3.4 + Units.inchesToMeters(14.0)), new InterpolatingDouble(307.5));
        map.put(new InterpolatingDouble(2.6 + Units.inchesToMeters(14.0)), new InterpolatingDouble(310.5));
        map.put(new InterpolatingDouble(2.0 + Units.inchesToMeters(14.0)), new InterpolatingDouble(314.5));
        map.put(new InterpolatingDouble(1.83 + Units.inchesToMeters(14.0)), new InterpolatingDouble(317.0));
        map.put(new InterpolatingDouble(1.5 + Units.inchesToMeters(14.0)), new InterpolatingDouble(318.5));
        map.put(new InterpolatingDouble(1.0 + Units.inchesToMeters(14.0)), new InterpolatingDouble(328.0));
    }

    @AutoLogOutput
    public double calculate(Pose2d pose) {
        double val = map.getInterpolated(new InterpolatingDouble(getSpeakerDistance(pose))).value;
        Logger.recordOutput("Interpolation Angle", val);
        return val;
    }

    public static double getSpeakerDistance(Pose2d pose) {
        Pose3d speaker = new Pose3d(red ? redSpeaker : blueSpeaker);
        // Get shooter pivot location relative to the center of the robot.
        Transform3d shooterPivot = new Transform3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(6.0), new Rotation3d());
        // Convert robot pose to shooter pivot pose relative to the field.
        Pose3d deltaPose = new Pose3d(pose).plus(shooterPivot);
        // Delta X and Delta Y variables to calculate distance from the speaker.
        double deltaX = deltaPose.getX() - speaker.getX(), deltaY = deltaPose.getY() - speaker.getY();
        // Calculate hypotenuse to get the distance from the speaker & the height of the speaker relative to the pivot.
        SmartDashboard.putNumber("sqrt", Math.sqrt(deltaX * deltaX + deltaY * deltaY));
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static void updateClosestInterpolationValue(Pose2d pose, double adj) {
        double dist = getSpeakerDistance(pose);
        // get surrounding keys for interpolation
        InterpolatingDouble topBound = map.ceilingKey(new InterpolatingDouble(dist));
        InterpolatingDouble bottomBound = map.floorKey(new InterpolatingDouble(dist));

        // if attempting interpolation at ends of tree, return the nearest data point
        InterpolatingDouble key;
        if (topBound == null && bottomBound == null) {
            return;
        } else if (topBound == null) {
            key = bottomBound;
        } else if (bottomBound == null) {
            key = topBound;
        } else {
            key = new InterpolatingDouble(
                    Math.abs(dist - bottomBound.value) > Math.abs(dist - topBound.value)
                            ? topBound.value : bottomBound.value);
        }

        map.replace(key, new InterpolatingDouble(map.get(key).value + adj));
        Logger.recordOutput("Adjusted " + key.value, map.get(key).value);
    }
}