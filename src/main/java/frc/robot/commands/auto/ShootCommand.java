package frc.robot.commands.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.kPitchMinimumAngle;
import static frc.robot.Constants.ShooterConstants.kSpeakerVelocity;

public class ShootCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private PoseSubsystem poseSubsystem = PoseSubsystem.getInstance();

    public ShootCommand() {
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.AUTO);
        m_shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        // m_shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.AUTO);
        m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
    }

    @Override
    public void execute() {
        super.execute();
        m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
        Pair<Double, Double> flapAngles = m_shooterSubsystem.getFlapCalculatedAngle(poseSubsystem.getPose());
        m_shooterSubsystem.setFlapPosition(flapAngles.getFirst(), flapAngles.getSecond());
        // m_shooterSubsystem.setPitchPosition(m_shooterSubsystem.getShooterCalculatedAngle(poseSubsystem.getPose()) + kPitchMinimumAngle);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.STRAIGHT);
        m_shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.STOP);
        // m_shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.DRIVE);
        m_shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // return m_shooterSubsystem.isOnTarget() && m_shooterSubsystem.shooterPitchArrived();
        return m_shooterSubsystem.isOnTarget();
    }
}