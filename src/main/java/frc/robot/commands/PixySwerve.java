package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PixycamSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class PixySwerve extends Command {
    private boolean check = false;
    private String[] objLocation;
    private final SwerveSubsystem swerve;
    private final PixycamSubsystem m_vision = PixycamSubsystem.getInstance();
    private Trigger btnPress; // controller button
    private final DoubleSupplier vX, vY, omega;
    private final PIDController rotationController = new PIDController(4, 0.0, 0.0);

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        check = false;
    }

    public PixySwerve(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                      DoubleSupplier omega, Trigger btnPressTrigger) {

        this.swerve = swerve;
        this.btnPress = btnPressTrigger;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;

        rotationController.enableContinuousInput(0, 2 * Math.PI);
        rotationController.setTolerance(Math.toRadians(2.0));

        addRequirements(swerve, m_vision);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xVelocity   = Math.pow(-vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(-vY.getAsDouble(), 3);
        double angVelocity = Math.pow(-omega.getAsDouble(), 3);

        try { // Call pixy values, convert them into a new array
            objLocation = m_vision.getPixyValue();
            SmartDashboard.putString("X", objLocation[0]);
            SmartDashboard.putString("Y", objLocation[1]);
            double angle = m_vision.calculateAngle();
            SmartDashboard.putNumber("Pixy Cam Angle", angle);
            if (btnPress.getAsBoolean()) {
                if ((Math.abs(m_vision.getPixyCenter()) > 8)) {
                    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), rotationController.calculate(angle, 0.0), true);
                } else {
                    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), 0, true);
                }
            } else {
                swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                        angVelocity * swerve.getSwerveController().config.maxAngularVelocity, true);
            }
        } catch (Exception e) {
            SmartDashboard.putString("Error2", "Method failed!");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return check;
    }
}