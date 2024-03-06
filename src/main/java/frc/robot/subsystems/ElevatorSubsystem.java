package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private static ElevatorSubsystem instance = getInstance();

    private final TalonFX climbMotor;
    private final DigitalInput leftClimbLimitSwitch, rightClimbLimitSwitch;

    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);

    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI,
    ElevatorConstants.kD, m_constraints, ElevatorConstants.kDt);

    private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    private double lastControlledPos = 0.0;
    //ToDo Must get proper encoder conversion prior to any tuning.
    public ElevatorSubsystem() {
        climbMotor = new TalonFX(Constants.Ports.kClimbPort);
        climbMotor.clearStickyFaults();
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        climbMotor.setInverted(true);

        m_controller.setTolerance(ElevatorConstants.climbGoalTolerance);

        leftClimbLimitSwitch = new DigitalInput(Constants.Ports.kLeftClimbLimitSwitch);
        rightClimbLimitSwitch = new DigitalInput(Constants.Ports.kRightClimbLimitSwitch);
    }

    public void setClimbSpeed(double speed) {
        // Verify set speed is attempting to go down, and whether it is touching one of the limit switches. If so, stop.
//        if(speed < 0 && (leftClimbLimitSwitch.get() || rightClimbLimitSwitch.get()))
//            stop();
//        else
            set(speed);
    }

    private void set(double speed) {
        lastControlledPos = climbMotor.getPosition().getValueAsDouble();
        climbMotor.set(speed);
    }

    public double holdPosition() {
        if(!(m_controller.getGoal().position == lastControlledPos))
            m_controller.setGoal(lastControlledPos);

        return m_controller.calculate(lastControlledPos)
                + m_feedforward.calculate(m_controller.getSetpoint().velocity);
    }

    public void stop() {
        climbMotor.set(0.0);
    }

     public void setGoalClimb() {
         m_controller.setGoal(ElevatorConstants.climbGoal);
     }

     public void setGoalDown() {
         m_controller.setGoal(ElevatorConstants.downGoal);
     }

     /**
      * Use the PID and Feedforward controllers to start moving towards the set goal.
      * @return Whether the climb motor is near its goal
      */
     public boolean climb() {
         climbMotor.setVoltage(
             m_controller.calculate(climbMotor.getPosition().getValueAsDouble())
                 + m_feedforward.calculate(m_controller.getSetpoint().velocity));
         return m_controller.atGoal();
     }

     public static ElevatorSubsystem getInstance() {
         if(instance == null)
             instance = new ElevatorSubsystem();
         return instance;
     }
}
