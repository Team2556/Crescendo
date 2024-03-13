package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.kPitchAmpPosition;

public class AmpScore extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public AmpScore() {
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.AMP);
        intakeSubsystem.set(1.0);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setPitchPosition(kPitchAmpPosition);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.DRIVE);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.shooterPitchArrived();
    }
}
