package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.kIntakeMaxSpeed;
import static frc.robot.Constants.kIntakeSlowSpeed;

public class IntakeAuto extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private double intakeSpeed = kIntakeMaxSpeed;

    public IntakeAuto() {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        super.execute();

        if (intakeSubsystem.getIntakeOutputCurrent() > 40) {
            intakeSpeed = kIntakeSlowSpeed;
        }
        if (intakeSubsystem.getIntakeOutputCurrent() < 20) {
            intakeSpeed = kIntakeMaxSpeed;
        }
        intakeSubsystem.setIntakeMotor(intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getIntakeLimitSwitch();
    }
}
