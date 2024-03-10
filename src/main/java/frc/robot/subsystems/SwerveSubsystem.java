// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;

import static frc.robot.Constants.SWERVE_MAX_SPEED;

public class SwerveSubsystem extends SubsystemBase {

    /**
    * Swerve drive object.
    */
    private final SwerveDrive swerveDrive;
    /**
    * Maximum speed of the robot in meters per second, used to limit acceleration.
    */
    public double maximumSpeed = SWERVE_MAX_SPEED;

    /**
    * Initialize {@link SwerveDrive} with the directory provided.
    *
    * @param directory Directory of swerve drive config files.
    */
    public SwerveSubsystem(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(1, 1);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3.89), 7.0, 1);
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(true);
        setupPathPlanner();
    }

    /**
    * Setup AutoBuilder for PathPlanner.
    */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(5,0,0),
                    // Rotation PID constants
                    2.0,
                    // Max module speed, in m/s
                    swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                    // Drive base radius in meters. Distance from robot center to the furthest module.
                    new ReplanningConfig()
                    // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /**
    * Construct the swerve drive.
    *
    * @param driveCfg      SwerveDriveConfiguration for the swerve.
    * @param controllerCfg Swerve Controller.
    */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    /**
    * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
    * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
    * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
    *
    * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
    *                      second. In robot-relative mode, positive x is towards the bow (front) and positive y is
    *                      towards port (left).  In field-relative mode, positive x is away from the alliance wall
    *                      (field North) and positive y is towards the left wall when looking through the driver station
    *                      glass (field West).
    * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
    *                      relativity.
    * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
    */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
            rotation,
            fieldRelative,
            false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
    * Drive the robot given a chassis field oriented velocity.
    *
    * @param velocity Velocity according to the field.
    */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
    * Drive according to the chassis robot oriented velocity.
    *
    * @param velocity Robot oriented {@link ChassisSpeeds}
    */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("x pose", getPose().getX());
        SmartDashboard.putNumber("y pose", getPose().getY());
        SmartDashboard.putNumber("theta pose", getPose().getRotation().getDegrees());
    }

    /**
    * Get the swerve drive kinematics object.
    *
    * @return {@link SwerveDriveKinematics} of the swerve drive.
    */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
    * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
    * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
    * keep working.
    *
    * @param initialHolonomicPose The pose to set the odometry to
    */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
    * Gets the current pose (position and rotation) of the robot, as reported by odometry.
    *
    * @return The robot's pose
    */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
    * Set chassis speeds with closed-loop velocity control.
    *
    * @param chassisSpeeds Chassis Speeds to set.
    */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
    * Post the trajectory to the field.
    *
    * @param trajectory The trajectory to post.
    */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
    * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
    */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
    * Sets the drive motors to brake/coast mode.
    *
    * @param brake True to set motors to brake mode, false for coast.
    */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
    * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
    *
    * @return The yaw angle
    */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
    * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
    * the angle of the robot.
    *
    * @param xInput   X joystick input for the robot to move in the X direction.
    * @param yInput   Y joystick input for the robot to move in the Y direction.
    * @param headingX X joystick which controls the angle of the robot.
    * @param headingY Y joystick which controls the angle of the robot.
    * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
    */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
    * Get the chassis speeds based on controller input of 1 joystick and one angle.
    *
    * @param xInput X joystick input for the robot to move in the X direction.
    * @param yInput Y joystick input for the robot to move in the Y direction.
    * @param angle  The angle in as a {@link Rotation2d}.
    * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
    */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
    * Gets the current field-relative velocity (x, y and omega) of the robot
    *
    * @return A ChassisSpeeds object of the current field-relative velocity
    */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
    * Gets the current velocity (x, y and omega) of the robot
    *
    * @return A {@link ChassisSpeeds} object of the current velocity
    */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
    * Get the {@link SwerveController} in the swerve drive.
    *
    * @return {@link SwerveController} from the {@link SwerveDrive}.
    */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
    * Get the {@link SwerveDriveConfiguration} object.
    *
    * @return The {@link SwerveDriveConfiguration} fpr the current drive.
    */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
    * Lock the swerve drive to prevent it from moving.
    */
    public void lock() {
        if(getFieldVelocity().vxMetersPerSecond < 0.5 && getFieldVelocity().vyMetersPerSecond < 0.5 && getFieldVelocity().omegaRadiansPerSecond < 0.5)
            swerveDrive.lockPose();
    }

    /**
    * Gets the current pitch angle of the robot, as reported by the imu.
    *
    * @return The heading as a {@link Rotation2d} angle
    */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }

    public void setMaximumSpeed(double speed) {
        swerveDrive.setMaximumSpeed(speed);
    }
}