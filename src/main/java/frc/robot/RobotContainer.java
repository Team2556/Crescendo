// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                            "swerve"));

    private final PhotonCamera camera = new PhotonCamera("photonVision");

    private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(camera, drivebase, drivebase::getPose);

    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

    private final PhotonSubsystem m_photonSubsystem = PhotonSubsystem.getInstance();

    // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
    XboxController driverXbox = new XboxController(0);
    XboxController operatorXbox = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        m_shooterSubsystem.setDefaultCommand(
      new ShootCommand(
        m_shooterSubsystem,
        operatorXbox.start(null),
        operatorXbox.povLeft(null),
        operatorXbox.povRight(null),
        operatorXbox.povUp(null),
        operatorXbox.povDown(null),
        operatorXbox::getLeftY,
        operatorXbox.leftTrigger(0.8, null),
        operatorXbox.rightTrigger(0.8, null),
        operatorXbox.x(null)
      )
    );

    // m_photonSubsystem.setDefaultCommand(
    //     new photonCommand(
    //         m_photonSubsystem
    //     )
    // );

        configureBindings();

        // AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
        //                                                                      () ->
        //                                                                          MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
        //                                                                      () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                                                                                                   OperatorConstants.LEFT_X_DEADBAND),
        //                                                                      () -> driverXbox.getRawAxis(2));

        TeleopDrive closedFieldRel = new TeleopDrive(
            drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), 
            () -> true);



        drivebase.setDefaultCommand(closedFieldRel);

        // shooter.setDefaultCommand(new ShooterControl());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
        new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
        new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
        new JoystickButton(driverXbox, 1).whileTrue(chaseTagCommand);
        // A
        // new JoystickButton(operatorXbox, 1).onTrue(new InstantCommand(() -> shooter.setState(ShooterState.AMP)))
        //         .onFalse(new InstantCommand(shooter::stop));
        // // B
        // new JoystickButton(operatorXbox, 2).onTrue(new InstantCommand(() -> shooter.setState(ShooterState.CLOSE)))
        //         .onFalse(new InstantCommand(shooter::stop));
        // // X
        // new JoystickButton(operatorXbox, 3).onTrue(new InstantCommand(() -> shooter.setState(ShooterState.MIDDLE)))
        //         .onFalse(new InstantCommand(shooter::stop));
        // // Y
        // new JoystickButton(operatorXbox, 4).onTrue(new InstantCommand(() -> shooter.setState(ShooterState.FAR)))
        //         .onFalse(new InstantCommand(shooter::stop));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("4 Note Leave");
        // An example command will be run in autonomous
        // return drivebase.getAutonomousCommand("Straight Test", true);
    }

    public void setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
