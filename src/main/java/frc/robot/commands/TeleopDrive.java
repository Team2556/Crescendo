// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Swerve drive command
 */
public class TeleopDrive extends Command {
    private final SwerveSubsystem  swerve;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final SwerveController controller;
    private static boolean red = false;
    public static boolean slowMode = false;
    public static boolean fieldOriented = true;
    private static final double slowModeScalar = 0.3;
    double xVelocity;
    double yVelocity;

    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                     DoubleSupplier omega) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.controller = swerve.getSwerveController();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        slowMode = false;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        alliance.ifPresent(value -> red = value.equals(DriverStation.Alliance.Red));
    }

    @Override
    public void execute() {
        if (red) {
            xVelocity   = Math.pow(vX.getAsDouble(), 3);
            yVelocity   = Math.pow(vY.getAsDouble(), 3);
        } else {
            xVelocity   = Math.pow(-vX.getAsDouble(), 3);
            yVelocity   = Math.pow(-vY.getAsDouble(), 3);
        }
        double angVelocity = Math.pow(-omega.getAsDouble(), 3);
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);
        xVelocity *= slowMode ? slowModeScalar : 1;
        yVelocity *= slowMode ? slowModeScalar : 1;
        angVelocity *= slowMode ? slowModeScalar : 1;
        // Drive using raw values.
        swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                angVelocity * controller.config.maxAngularVelocity,
                fieldOriented);
    }
}