package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final DoubleSupplier m_leftStick;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier leftStick) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_leftStick = leftStick;

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
        //ToDo Test PID + FF Control to hold the climb's position.
//        if(m_leftStick.getAsDouble() > Constants.ElevatorConstants.kClimbDeadband) {
//            m_elevatorSubsystem.set(m_leftStick.getAsDouble() * Constants.ElevatorConstants.kMaxSpeed);
//        } else
//            m_elevatorSubsystem.holdPosition();

        m_elevatorSubsystem.set(m_leftStick.getAsDouble() * Constants.ElevatorConstants.kMaxSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_elevatorSubsystem.stop();
    }
}
