// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;

import java.io.File;
import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.Constants.SLOW_MAX_SPEED;
import static frc.robot.Constants.SWERVE_MAX_SPEED;
import static frc.robot.Constants.ShooterConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Instances of robot subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                            "swerve"));
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final PoseSubsystem m_poseSubsystem = PoseSubsystem.getInstance();
    // Drive controllers
    CommandXboxController driverXbox = new CommandXboxController(0);
    CommandXboxController operatorXbox = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        TeleopDrive closedFieldRel = new TeleopDrive(
            drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

        m_shooterSubsystem.setDefaultCommand(new ShootCommand(operatorXbox.rightTrigger(0.5), driverXbox.rightBumper()));
        m_intakeSubsystem.setDefaultCommand(new IntakeControlCommand(driverXbox::getRightTriggerAxis, driverXbox::getLeftTriggerAxis));
        m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(() -> -operatorXbox.getLeftY(), () -> operatorXbox.getLeftTriggerAxis()));

        m_poseSubsystem.setDefaultCommand(new PoseUpdateCommand(m_poseSubsystem));
        m_poseSubsystem.initialize(drivebase, new PhotonCamera("photonVision"));

        drivebase.setDefaultCommand(closedFieldRel);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        driverXbox.leftStick().whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
        driverXbox.a().onTrue(new InstantCommand(() -> {
            drivebase.setMaximumSpeed(TeleopDrive.slowMode ? SLOW_MAX_SPEED : SWERVE_MAX_SPEED);
            TeleopDrive.slowMode = !TeleopDrive.slowMode;
        }));
        driverXbox.b().onTrue(new InstantCommand(() -> TeleopDrive.fieldOriented = !TeleopDrive.fieldOriented));
        driverXbox.y().onTrue((new InstantCommand(drivebase::zeroGyro)));

        //ToDo Get pose for red side
        driverXbox.povUp().onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(3.3, 4.8, Rotation2d.fromDegrees(0.0)),
                        new PathConstraints(
                                2.0, 1.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        //ToDo Get pose for red side
        driverXbox.povLeft().onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(1.8, 7.6, Rotation2d.fromDegrees(0.0)),
                        new PathConstraints(
                                2.0, 1.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        //ToDo Get pose for red side
        driverXbox.povRight().onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(300)),
                        new PathConstraints(
                                2.0, 1.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        operatorXbox.start().onTrue(new InstantCommand(m_shooterSubsystem::resetFlaps));
        operatorXbox.rightBumper().onTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterState(ShooterState.SPEAKER)));
        operatorXbox.leftBumper().onTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterState(ShooterState.AMP)));
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            m_shooterSubsystem.setShooterState(ShooterState.INTAKE);
            m_shooterSubsystem.setFlapState(FlapState.INTAKE);
        }));
        operatorXbox.y().onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.AUTO)));
        operatorXbox.x().onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.STRAIGHT)));
//        operatorXbox.b().onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.TEST)));

        AtomicBoolean shot = new AtomicBoolean(false);
        // Command to execute when right bumper is pressed
        Command pressCommand = new SequentialCommandGroup(
                new WaitUntilCommand(m_shooterSubsystem::shouldShoot), // Wait until shooter is ready
                new IntakeSetCommand(0.4).withTimeout(1.0), // Run intake command
                new InstantCommand(() -> shot.set(true)) // Set 'shot' to true
        );

        // Command to execute when right bumper is released
        Command releaseCommand = new SequentialCommandGroup(
                // Run intake command and set 'shot' to false
                new IntakeSetCommand(0.4).withTimeout(1.0)
                        .alongWith(new InstantCommand(() -> shot.set(false)))
        );

        driverXbox.rightBumper().onTrue(pressCommand).onFalse(releaseCommand);

        Command ampScore = new SequentialCommandGroup(
                new RunCommand(() -> {
                    m_shooterSubsystem.setFlapState(FlapState.STRAIGHT);
                    m_shooterSubsystem.setFlapPosition(kLeft90, kRight90);
                }, m_shooterSubsystem),
                new WaitUntilCommand(() -> m_shooterSubsystem.flapsArrived(kLeft90, kRight90)),
                new RunCommand(() -> m_shooterSubsystem.setPitchPosition(kPitchVerticalPosition), m_shooterSubsystem),
                new WaitUntilCommand(m_shooterSubsystem::shooterPitchArrived),
                new RunCommand(() -> m_shooterSubsystem.setPitchPosition(kPitchAmpPosition), m_shooterSubsystem)
                        .alongWith(new IntakeSetCommand(0.4).withTimeout(1.0)),
                new WaitCommand(2.0)
        );

        driverXbox.leftBumper().onTrue(ampScore);
   }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("StartAmp3Leave");
    }
}