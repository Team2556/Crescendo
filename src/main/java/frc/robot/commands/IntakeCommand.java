package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.OperatorConstants.LEFT_TRIGGER_DEADBAND;
import static frc.robot.Constants.OperatorConstants.RIGHT_TRIGGER_DEADBAND;
import static frc.robot.Constants.ShooterConstants.kIntakeMaxSpeed;
import static frc.robot.Constants.ShooterConstants.kOuttakeMaxSpeed;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;
    private final DoubleSupplier m_rightTrigger, m_leftTrigger;
    public IntakeCommand(IntakeSubsystem subsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        m_subsystem = subsystem;
        m_rightTrigger = rightTrigger;
        m_leftTrigger = leftTrigger;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_subsystem.stop();
    }

    @Override
    public void execute() {
        super.execute();
        if(m_rightTrigger.getAsDouble() > RIGHT_TRIGGER_DEADBAND)
            m_subsystem.setIntakeMotor(kIntakeMaxSpeed * m_rightTrigger.getAsDouble());
        else if(m_leftTrigger.getAsDouble() > LEFT_TRIGGER_DEADBAND)
            m_subsystem.setIntakeMotor(kOuttakeMaxSpeed);
        else
            m_subsystem.stop();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_subsystem.stop();
    }
}