package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCommand extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final double speed;

    public IntakeSetCommand(double speed) {
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putBoolean("Waiting to shoot", false);
        intakeSubsystem.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intakeSubsystem.stop();
    }
}
