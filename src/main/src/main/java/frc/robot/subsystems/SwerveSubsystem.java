// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  SwerveDriveKinematics             kinematics;
  SwerveDriveOdometry               odometry;
  AHRS                              gyro;
  frc.robot.subsystems.SwerveModule[] swerveModules;

  private final SwerveDrive swerveDrive;
  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SwerveSubsystem(File directory) {
    

    // swerveModules[0] = new frc.robot.subsystems.SwerveModule(
    //   Constants.SwerveModuleConstants.kDriveLF, 
    //   Constants.SwerveModuleConstants.kTurnLF, 
    //   Constants.SwerveModuleConstants.kCANcoderLF
    // );
    // swerveModules[1] = new frc.robot.subsystems.SwerveModule(
    //   Constants.SwerveModuleConstants.kDriveRF, 
    //   Constants.SwerveModuleConstants.kTurnRF, 
    //   Constants.SwerveModuleConstants.kCANcoderRF
    // );
    // swerveModules[2] = new frc.robot.subsystems.SwerveModule(
    //   Constants.SwerveModuleConstants.kDriveLR, 
    //   Constants.SwerveModuleConstants.kTurnLR, 
    //   Constants.SwerveModuleConstants.kCANcoderLR
    // );
    // swerveModules[3] = new frc.robot.subsystems.SwerveModule(
    //   Constants.SwerveModuleConstants.kDriveRR, 
    //   Constants.SwerveModuleConstants.kTurnRR, 
    //   Constants.SwerveModuleConstants.kCANcoderRR
    // );

    // kinematics = new SwerveDriveKinematics(
    //   new Translation2d(Units.inchesToMeters(Constants.SwerveModuleConstants.lFX),
    //       Units.inchesToMeters(Constants.SwerveModuleConstants.lFY)),
    //   new Translation2d(Units.inchesToMeters(Constants.SwerveModuleConstants.rFX),
    //       Units.inchesToMeters(Constants.SwerveModuleConstants.rFY)),
    //   new Translation2d(Units.inchesToMeters(Constants.SwerveModuleConstants.lRX),
    //       Units.inchesToMeters(Constants.SwerveModuleConstants.lRY)),
    //   new Translation2d(Units.inchesToMeters(Constants.SwerveModuleConstants.rRX),
    //       Units.inchesToMeters(Constants.SwerveModuleConstants.rRY))
    // );

    gyro = new AHRS();

    // odometry = new SwerveDriveOdometry(
    //   kinematics, 
    //   gyro.getRotation2d(), 
    //   new SwerveModulePosition[] {
    //     new SwerveModulePosition(), 
    //     new SwerveModulePosition(),
    //     new SwerveModulePosition(),
    //     new SwerveModulePosition()
    //   }, 
    //   new Pose2d(
    //     0, 
    //     0, 
    //     new Rotation2d()
    //   )
    // );

    
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    setUpPathPlanner();
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  public void setUpPathPlanner() {
      AutoBuilder.configureHolonomic(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                           new PIDConstants(5.0, 0.0, 0.0),
                                           // Translation PID constants
                                           new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                                            swerveDrive.swerveController.config.headingPIDF.i,
                                                            swerveDrive.swerveController.config.headingPIDF.d),
                                           // Rotation PID constants
                                           4.5,
                                           // Max module speed, in m/s
                                           swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                           // Drive base radius in meters. Distance from robot center to furthest module.
                                           new ReplanningConfig()
                                           // Default path replanning config. See the API for the options here
          ),
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
          },
          this // Reference to this subsystem to set requirements
                                    );
    }

  // public void drive() {
  //   ChassisSpeeds testSpeeds = new ChassisSpeeds(
  //     Units.inchesToMeters(14), 
  //     Units.inchesToMeters (4), 
  //     Units.degreesToRadians(30)
  //   );

  //   SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);

  //   swerveModules[0].setState(swerveModuleStates[0]);
  //   swerveModules[1].setState(swerveModuleStates[1]);
  //   swerveModules[2].setState(swerveModuleStates[2]);
  //   swerveModules[3].setState(swerveModuleStates[3]);  
  // }

  // public SwerveModulePosition[] getCurrentModulePositions() {
  //   return new SwerveModulePosition[] {
  //     new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()),
  //     new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()),
  //     new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()),
  //     new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())
  //   };
  // }

  public void gyroReset() {
    swerveDrive.zeroGyro();
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3);
      double yInput = Math.pow(translationY.getAsDouble(), 3);
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
        headingX.getAsDouble(), 
        headingY.getAsDouble(), 
        swerveDrive.getYaw().getRadians(), 
        swerveDrive.getMaximumVelocity()));
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // odometry.update(gyro.getRotation2d(), getCurrentModulePositions());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

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

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  
}
