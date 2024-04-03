package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.kPitchSpeakerPosition;
import static frc.robot.Constants.ShooterConstants.kSpeakerVelocity;

public class ShootPreload extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ShootPreload() {
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.SPEAKER);
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
        shooterSubsystem.setPitchPosition(kPitchSpeakerPosition);
        intakeSubsystem.set(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.STRAIGHT);
        shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}