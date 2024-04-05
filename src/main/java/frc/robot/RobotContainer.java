// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;

import java.io.File;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.Constants.*;
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
    private final PixycamSubsystem m_pixySubsystem = PixycamSubsystem.getInstance();
    private final SendableChooser<Command> autoChooser;
    // Drive controllers
    CommandXboxController driverXbox = new CommandXboxController(0);
    CommandXboxController operatorXbox = new CommandXboxController(1);

    Command shootPreload;
    Command intake;
    Command shoot;

    public static boolean shouldRumble = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        shootPreload = new SequentialCommandGroup(
                new ResetFlaps(),
                new PreparePreload(),
                new CanShootPreload(),
                new ShootPreload().withTimeout(1.5)
        );

        NamedCommands.registerCommand("Shoot Preload", shootPreload);

        intake = new IntakeAuto();

        NamedCommands.registerCommand("Intake", intake.withTimeout(0.5));

        shoot = new SequentialCommandGroup(
                new ShootCommand(),
                new IntakeSetCommand(1.0).withTimeout(0.3),
                new InstantCommand(() -> {
                    m_shooterSubsystem.setFlapState(FlapState.STRAIGHT);
                    m_shooterSubsystem.setShooterState(ShooterState.SPEAKER);
                    m_shooterSubsystem.setPitchState(PitchState.DRIVE);
                    m_intakeSubsystem.stop();
                }, m_intakeSubsystem, m_shooterSubsystem)
        );

        NamedCommands.registerCommand("Shoot", shoot);

        TeleopDrive closedFieldRel = new TeleopDrive(
            drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

        PixySwerve pixySwerve = new PixySwerve(drivebase,
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox.start());

//        m_shooterSubsystem.setDefaultCommand(new ShooterAngleTest());

        m_shooterSubsystem.setDefaultCommand(new ShootControlCommand(operatorXbox.rightTrigger(0.5), driverXbox.rightBumper()));
        m_intakeSubsystem.setDefaultCommand(new IntakeControlCommand(driverXbox::getRightTriggerAxis, driverXbox::getLeftTriggerAxis));
        m_elevatorSubsystem.setDefaultCommand(new ElevatorCommand(() -> -operatorXbox.getLeftY(), () -> operatorXbox.getLeftTriggerAxis()));
        m_poseSubsystem.setDefaultCommand(new PoseUpdateCommand(m_poseSubsystem));
        m_poseSubsystem.initialize(drivebase, new PhotonCamera("photonVision"));

//        drivebase.setDefaultCommand(pixySwerve);
        drivebase.setDefaultCommand(closedFieldRel);

        autoChooser = AutoBuilder.buildAutoChooser();

        new PathPlannerAuto("StartCenter1Middle1");
        new PathPlannerAuto("StartCenter2Leave");
        new PathPlannerAuto("StartAmp1Leave");
        new PathPlannerAuto("StartAmp2Leave");
//        new PathPlannerAuto("StartAmp3Leave");
        new PathPlannerAuto("StartSource2");
        new PathPlannerAuto("StartAway2");
        new PathPlannerAuto("StartCenterTo4Note");
        new PathPlannerAuto("Amp1Middle1");

        SmartDashboard.putData("Auto Mode", autoChooser);

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
        driverXbox.y().onTrue((new InstantCommand(drivebase::zeroGyroWithAlliance)));

        driverXbox.x().whileTrue(drivebase.aimAtSpeaker(2));

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        Pose2d speakerPose = blueSpeakerScore, ampPose = blueAmp, intakePose = blueIntake, subwooferPose = blueSubwoofer;
        if(alliance.isPresent()) {
            speakerPose = alliance.get().equals(DriverStation.Alliance.Red) ? redSpeakerScore : blueSpeakerScore;
            ampPose = alliance.get().equals(DriverStation.Alliance.Red) ? redAmp : blueAmp;
            intakePose = alliance.get().equals(DriverStation.Alliance.Red) ? redIntake : blueIntake;
            subwooferPose = alliance.get().equals(DriverStation.Alliance.Red) ? redSubwoofer : blueSubwoofer;
        }

        driverXbox.povUp().onTrue(
                AutoBuilder.pathfindToPose(
                        speakerPose,
                        new PathConstraints(
                                3.0, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        driverXbox.povLeft().onTrue(
                AutoBuilder.pathfindToPose(
                        ampPose.plus(new Transform2d(0, Units.inchesToMeters(-12.0), new Rotation2d())),
                        new PathConstraints(
                                1.0, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                ).andThen(AutoBuilder.pathfindToPose(
                        ampPose,
                        new PathConstraints(
                                1.0, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                ))
        );

        driverXbox.povRight().onTrue(
                AutoBuilder.pathfindToPose(
                        intakePose,
                        new PathConstraints(
                                3.0, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        driverXbox.povDown().onTrue(
                AutoBuilder.pathfindToPose(
                        subwooferPose,
                        new PathConstraints(
                                3.0, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ),
                        0,
                        0
                )
        );

        operatorXbox.start().onTrue(new InstantCommand(m_shooterSubsystem::resetFlaps));
        operatorXbox.rightBumper().onTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterState(ShooterState.SPEAKER)));
        operatorXbox.leftBumper().onTrue(new InstantCommand(() -> {
            m_shooterSubsystem.setShooterState(ShooterState.AMP);
            m_shooterSubsystem.setPitchState(PitchState.AMP);
            m_shooterSubsystem.setFlapState(FlapState.STRAIGHT);
        }));
        operatorXbox.a().onTrue(new InstantCommand(() -> {
            m_shooterSubsystem.setShooterState(ShooterState.INTAKE);
            m_shooterSubsystem.setFlapState(FlapState.INTAKE);
            m_shooterSubsystem.setPitchState(PitchState.INTAKE);
        }));
        operatorXbox.b().onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.SIDE)));
        operatorXbox.x().onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.STRAIGHT)));
        operatorXbox.y().onTrue(new InstantCommand(() -> {
            m_shooterSubsystem.setFlapState(FlapState.AUTO);
            m_shooterSubsystem.setPitchState(PitchState.AUTO);
            m_shooterSubsystem.setShooterState(ShooterState.SPEAKER);
        }));
        operatorXbox.povUp().onTrue(new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.VERTICAL)));
        operatorXbox.povDown().onTrue(new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.DRIVE)));
        operatorXbox.povLeft().onTrue(new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.AMP)));
        operatorXbox.povRight().onTrue(new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.SPEAKER)));

        new Trigger(() -> shouldRumble).onTrue(rumble(GenericHID.RumbleType.kBothRumble, 0.5));

        AtomicBoolean shot = new AtomicBoolean(false);
        // Command to execute when right bumper is pressed
        Command pressCommand = new SequentialCommandGroup(
                new WaitUntilCommand(m_shooterSubsystem::shouldShoot), // Wait until shooter is ready
                new IntakeSetCommand(1.0).withTimeout(1.0)
                        .alongWith(rumble(GenericHID.RumbleType.kBothRumble, 0.5)), // Run intake command
                new InstantCommand(() -> shot.set(true)), // Set 'shot' to true
                new WaitCommand(3.0).andThen(
                       new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.DRIVE)),
                        new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.SIDE)) 
                )
        );

        // Command to execute when right bumper is released
        Command releaseCommand = new SequentialCommandGroup(
                // Run intake command and set 'shot' to false
                new IntakeSetCommand(1.0).withTimeout(1.0)
                        .alongWith(new InstantCommand(() -> shot.set(false)))
        );

        driverXbox.rightBumper().onTrue(pressCommand).onFalse(releaseCommand.andThen(
                new InstantCommand(() -> m_shooterSubsystem.setPitchState(PitchState.DRIVE)),
                new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.SIDE))));

//        Command ampScore = new SequentialCommandGroup(
//                new AmpVertical(),
//                new AmpScore()
//        );
//
//        driverXbox.leftBumper().onTrue(ampScore);
   }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command rumble(GenericHID.RumbleType rumbleType, double strength) {
        return Commands.runOnce(
                        () -> {
                            driverXbox.getHID().setRumble(rumbleType, strength);
                            operatorXbox.getHID().setRumble(rumbleType, strength);
                        })
                .andThen(Commands.waitSeconds(0.3))
                .finallyDo(
                        () -> {
                            driverXbox.getHID().setRumble(rumbleType, 0);
                            operatorXbox.getHID().setRumble(rumbleType, 0);
                        });
    }
}