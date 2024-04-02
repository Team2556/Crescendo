package frc.robot.commands.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterInterpolation;

import java.util.Optional;

import static frc.robot.Constants.ShooterConstants.kSpeakerVelocity;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final PoseSubsystem poseSubsystem = PoseSubsystem.getInstance();

    private ShooterInterpolation interpolation;

    public ShootCommand() {
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        alliance.ifPresent(value -> ShooterSubsystem.red = value.equals(DriverStation.Alliance.Red));
        interpolation = new ShooterInterpolation(ShooterSubsystem.red);

        m_shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.AUTO);
        m_shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        m_shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.AUTO);
        m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
    }

    @Override
    public void execute() {
        super.execute();
        m_shooterSubsystem.setShooterVelocity(kSpeakerVelocity);
        Pair<Double, Double> flapAngles = m_shooterSubsystem.getFlapCalculatedAngle(poseSubsystem.getPose());
        m_shooterSubsystem.setFlapPosition(flapAngles.getFirst(), flapAngles.getSecond());
        m_shooterSubsystem.setPitchPosition(interpolation.calculate(poseSubsystem.getPose()));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_shooterSubsystem.setFlapState(Constants.ShooterConstants.FlapState.STRAIGHT);
        m_shooterSubsystem.setShooterState(Constants.ShooterConstants.ShooterState.SPEAKER);
        m_shooterSubsystem.setPitchState(Constants.ShooterConstants.PitchState.DRIVE);
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.isOnTarget() && m_shooterSubsystem.shooterPitchArrived();
    }
}