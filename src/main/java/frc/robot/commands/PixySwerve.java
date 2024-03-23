package frc.robot.commands;

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
    private float[] XandY = { 0, 0 };
    private final SwerveSubsystem swerve;
    private final PixycamSubsystem m_vision;
    private Trigger btnPress; // controller button
    private Translation2d translation = new Translation2d(0, 0);

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        check = false;
    }

    public PixySwerve(PixycamSubsystem pixySubsystem, SwerveSubsystem swerve, Trigger btnPressTrigger) {
        this.swerve = swerve;
        m_vision = pixySubsystem;
        this.btnPress = btnPressTrigger;
        addRequirements(swerve, pixySubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        try { // Call pixy values, convert them into a new array
            objLocation = m_vision.getPixyValue();
            if (btnPress.getAsBoolean()) {
                SmartDashboard.putString("X", objLocation[0]);
                SmartDashboard.putString("Y", objLocation[1]);
                XandY[1] = Float.parseFloat(objLocation[1]);
                SmartDashboard.putNumber("Number array Y", XandY[1]);
                if ((Math.abs(m_vision.getPixyCenter()) > 8)) {
                    swerve.drive(translation, m_vision.calculateAngle(), true);
                } else {
                    swerve.drive(translation, 0, true);
                }

            } else {
                SmartDashboard.getString("error", "Button not found");
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