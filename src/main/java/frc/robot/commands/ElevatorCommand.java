package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final DoubleSupplier m_leftStick, m_leftTrigger;

    public ElevatorCommand(DoubleSupplier leftStick, DoubleSupplier leftTrigger) {
        m_leftStick = leftStick;
        m_leftTrigger = leftTrigger;

        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_elevatorSubsystem.stop();
    }

    @Override
    public void execute() {
        super.execute();
        double stick = m_leftStick.getAsDouble();
        stick = stick > 0.2 ? stick : 0.0;
        double speed = stick * Constants.kElevatorMaxSpeed;
        if (m_leftTrigger.getAsDouble() > 0.5) {
            m_elevatorSubsystem.setClimbSpeedAnd(speed);
        } else {
            m_elevatorSubsystem.setClimbSpeedOr(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_elevatorSubsystem.stop();
    }
}