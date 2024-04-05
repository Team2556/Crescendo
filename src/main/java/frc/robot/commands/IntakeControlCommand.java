package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.common.hardware.VisionLEDMode;

import static frc.robot.Constants.OperatorConstants.LEFT_TRIGGER_DEADBAND;
import static frc.robot.Constants.OperatorConstants.RIGHT_TRIGGER_DEADBAND;
import static frc.robot.Constants.kIntakeMaxSpeed;
import static frc.robot.Constants.kIntakeSlowSpeed;
import static frc.robot.Constants.kOuttakeMaxSpeed;

public class IntakeControlCommand extends Command {
    private final IntakeSubsystem m_subsystem = IntakeSubsystem.getInstance();
    private final PoseSubsystem m_pose = PoseSubsystem.getInstance();
    private final DoubleSupplier m_rightTrigger, m_leftTrigger;
    private double intakeSpeed = kIntakeMaxSpeed;

    public IntakeControlCommand(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        m_rightTrigger = rightTrigger;
        m_leftTrigger = leftTrigger;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        intakeSpeed = kIntakeMaxSpeed;
        m_subsystem.stop();
    }

    @Override
    public void execute() {
        super.execute();
        if (m_subsystem.getIntakeOutputCurrent() > 40) {
            intakeSpeed = kIntakeSlowSpeed;
            m_pose.photonCamera.setLED(VisionLEDMode.kBlink);
            RobotContainer.shouldRumble = true;
        }
        if (m_subsystem.getIntakeOutputCurrent() < 20) {
            intakeSpeed = kIntakeMaxSpeed;
            RobotContainer.shouldRumble = false;
        }
        SmartDashboard.putNumber("Intake Set Speed", intakeSpeed);
        if(m_rightTrigger.getAsDouble() > RIGHT_TRIGGER_DEADBAND)
            m_subsystem.setIntakeMotor(intakeSpeed * m_rightTrigger.getAsDouble());
        else if(m_leftTrigger.getAsDouble() > LEFT_TRIGGER_DEADBAND)
            m_subsystem.set(kOuttakeMaxSpeed);
        else
            m_subsystem.stop();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_subsystem.stop();
    }
}