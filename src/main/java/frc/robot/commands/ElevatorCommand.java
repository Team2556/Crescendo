package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final DoubleSupplier m_leftStick, m_leftTrigger;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier leftStick, DoubleSupplier leftTrigger) {
        m_elevatorSubsystem = elevatorSubsystem;
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
        double speed = m_leftStick.getAsDouble() * Constants.kElevatorMaxSpeed;
        SmartDashboard.putNumber("Climb Speed", speed);
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
