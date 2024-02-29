// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Constants.ShooterConstants.FlapState;

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
    // Drive controllers
    XboxController driverXbox = new XboxController(0);
    XboxController operatorXbox = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        TeleopDrive closedFieldRel = new TeleopDrive(
            drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> true);

        m_shooterSubsystem.setDefaultCommand(new ShootCommand(m_shooterSubsystem));

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
        new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
        new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));

        new JoystickButton(operatorXbox, 8).onTrue(new InstantCommand(m_shooterSubsystem::resetFlaps));
        new JoystickButton(operatorXbox, 6).onTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterState(ShooterState.SPEAKER)));
        new JoystickButton(operatorXbox, 5).onTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterState(ShooterState.AMP)));
        new JoystickButton(operatorXbox, 4).onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.AUTO)));
        new JoystickButton(operatorXbox, 3).onTrue(new InstantCommand(() -> m_shooterSubsystem.setFlapState(FlapState.STRAIGHT)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("4 Note Leave");
    }
}