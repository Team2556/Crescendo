// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    private final static ShooterSubsystem instance = ShooterSubsystem.getInstance();
    // Shooter motors
    private final CANSparkFlex leftShooter, rightShooter;
    // Flap motors
    public final CANSparkMax leftFlap, rightFlap, shooterPitch;
    // Limit switches for flaps
    public final DigitalInput leftLimitSwitch, rightLimitSwitch, forwardLimitSwitch, backwardLimitSwitch;
    // Spark max/flex encoders
    private final RelativeEncoder lFlapEncoder, rFlapEncoder, leftShooterEncoder, rightShooterEncoder;
    private final SparkAbsoluteEncoder shooterPitchEncoder;
    // Spark max/flex pid controllers
    private final SparkPIDController leftShooterPID, rightShooterPID, lFlapPID, rFlapPID;
    private final PIDController shooterPitchPID = new PIDController(pitchShooterPIDF.p, pitchShooterPIDF.i, pitchShooterPIDF.d);
    private final ArmFeedforward shooterFeedforward = new ArmFeedforward(kS, kG, kV);
    // Whether flaps have been zeroed with their limit switches.
    public boolean leftHomeFlag = false, rightHomeFlag = false;
    // Target velocity instance variable.
    private double targetVelocity = 0;
    // Rolling average instance variable.
    private int rollingAverage = 0;
    private ShooterState shooterState = ShooterState.STOP;
    private FlapState flapState = FlapState.NONE;
    private PitchState pitchState = PitchState.NONE;

    /**
     * Constructor to handle the initialization and configuration of motors,
     * encoders, pid controllers, and limit switches.
     */
    public ShooterSubsystem() {
        leftShooter = new CANSparkFlex(Ports.kLeftShooterPort, MotorType.kBrushless);
        rightShooter = new CANSparkFlex(Ports.kRightShooterPort, MotorType.kBrushless);

        leftFlap = new CANSparkMax(Ports.kLeftFlap, MotorType.kBrushless);
        rightFlap = new CANSparkMax(Ports.kRightFlap, MotorType.kBrushless);

        shooterPitch = new CANSparkMax(Ports.kShooterPitch, MotorType.kBrushless);

        leftLimitSwitch = new DigitalInput(Ports.kLeftFlapLimitSwitch);
        rightLimitSwitch = new DigitalInput(Ports.kRightFlapLimitSwitch);

        forwardLimitSwitch = new DigitalInput(Ports.kForwardLimitSwitch);
        backwardLimitSwitch = new DigitalInput(Ports.kBackwardLimitSwitch);

        leftShooter.clearFaults();
        rightShooter.clearFaults();

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);

        leftShooterEncoder = leftShooter.getEncoder();
        rightShooterEncoder = rightShooter.getEncoder();

        leftShooterPID = leftShooter.getPIDController();
        rightShooterPID = rightShooter.getPIDController();

        leftShooterPID.setFeedbackDevice(leftShooterEncoder);
        rightShooterPID.setFeedbackDevice(rightShooterEncoder);

        leftShooterPID.setP(leftShooterPIDF.p);
        leftShooterPID.setI(leftShooterPIDF.i);
        leftShooterPID.setIZone(leftShooterPIDF.iz);
        leftShooterPID.setD(leftShooterPIDF.d);
        leftShooterPID.setFF(leftShooterPIDF.f);
        leftShooterPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

        rightShooterPID.setP(rightShooterPIDF.p);
        rightShooterPID.setI(rightShooterPIDF.i);
        rightShooterPID.setIZone(rightShooterPIDF.iz);
        rightShooterPID.setD(rightShooterPIDF.d);
        rightShooterPID.setFF(rightShooterPIDF.f);
        rightShooterPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

        lFlapEncoder = leftFlap.getEncoder();
        rFlapEncoder = rightFlap.getEncoder();

        lFlapPID = leftFlap.getPIDController();
        rFlapPID = rightFlap.getPIDController();

        lFlapPID.setP(leftFlapPIDF.p);
        lFlapPID.setI(leftFlapPIDF.i);
        lFlapPID.setIZone(leftFlapPIDF.iz);
        lFlapPID.setD(leftFlapPIDF.d);
        lFlapPID.setFF(leftFlapPIDF.f);
        lFlapPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

        rFlapPID.setP(rightFlapPIDF.p);
        rFlapPID.setI(rightFlapPIDF.i);
        rFlapPID.setIZone(rightFlapPIDF.iz);
        rFlapPID.setD(rightFlapPIDF.d);
        rFlapPID.setFF(rightFlapPIDF.f);
        rFlapPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

        shooterPitch.restoreFactoryDefaults();
        shooterPitch.clearFaults();

        shooterPitchEncoder = shooterPitch.getAbsoluteEncoder();
        shooterPitchEncoder.setInverted(true);
        shooterPitchEncoder.setPositionConversionFactor(360.0);
        shooterPitchEncoder.setVelocityConversionFactor(6.0);

        shooterPitchPID.enableContinuousInput(0, 360.0);
        shooterPitchPID.setTolerance(1.0);

        stop();

        leftShooter.burnFlash();
        rightShooter.burnFlash();

        leftFlap.burnFlash();
        rightFlap.burnFlash();

        leftHomeFlag = false;
        rightHomeFlag = false;
    }

    /**
     * Set shooter velocity.
     * Updates targetVelocity.
     * @param velocity target velocity.
     */
    public void setShooterVelocity(double velocity) {
        targetVelocity = velocity;
        leftShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
        rightShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
    }

    /**
     * Set shooter velocity.
     * Updates targetVelocity.
     * @param leftVelo Left target velocity.
     * @param rightVelo Right target velocity
     */
    public void setShooterVelocity(double leftVelo, double rightVelo) {
        targetVelocity = (leftVelo + rightVelo) / 2.0;
        leftShooterPID.setReference(leftVelo, CANSparkBase.ControlType.kVelocity);
        rightShooterPID.setReference(rightVelo, CANSparkBase.ControlType.kVelocity);
    }

    /**
     * Sets flap to position.
     * @param leftPosition position of left flap.
     * @param rightPosition position of right flap.
     */

    public void setFlapPosition(double leftPosition, double rightPosition) {
        lFlapPID.setReference(leftPosition, CANSparkBase.ControlType.kPosition);
        rFlapPID.setReference(rightPosition, CANSparkBase.ControlType.kPosition);
    }

    /**
     * Set speed based off of percent.
     * @param speed speed percent from -1 to 1
     */
    public void setSpeed(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    /**
     * Set the shooter angle.
     * @param position Position in degrees to set the shooter angle to.
     */
    //ToDo Add angle bounding to verify the inputted position is within the physically possible range.
    public void setPitchPosition(double position) {
        double pid = shooterPitchPID.calculate(getShooterPitch(), position);
        if((!forwardLimitSwitch.get() && pid < 0) || (!backwardLimitSwitch.get() && pid > 0)) {
            shooterPitch.setVoltage(0.0);
            return;
        }
        // Convert position to the proper frame of reference for the feedforward cosine.
        double pos = position - kPitchMinimumAngle;
        if(pos < 0) {
            pos = position + 90.0;
        }
        double ff = shooterFeedforward.calculate(Math.toRadians(pos), 0);
        shooterPitch.setVoltage(pid + ff);
    }

    public void stopPitch() {
        shooterPitch.setVoltage(0.0);
    }

    /**
     * Stop shooter motors.
     */
    public void stop() {
        setSpeed(0);
    }

    /**
     * Get average velocity between the left and right shooter motors.
     * @return Average velocity of the shooters.
     */
    public double getVelocity() {
        return (leftShooterEncoder.getVelocity() + rightShooterEncoder.getVelocity()) / 2.0;
    }

    /**
     * Get whether the current velocity is close enough to the target velocity based off a velocity error constant.
     * @return Whether the current velocity is near the target velocity.
     */
    public boolean isOnTarget() {
        return Math.abs(targetVelocity - getVelocity()) <= ShooterConstants.kVelocityTolerance;
    }

    /**
     * Check how often the shooter is 'on target' by being near the target velocity.
     * @param percent Minimum of the average to be acceptable.
     * @return Whether the average times on target is greater than the minimum input.
     */
    public boolean isOnTargetAverage(int percent) {
        if (percent > 10) {
            percent = 10;
        } else if (percent < 0) {
            percent = 0;
        }

        return rollingAverage > percent;
    }

    /**
     * Move the flaps until they reach the limit switch to zero their positions.
     */
    public void flapHome() {
        if (rightHomeFlag && leftHomeFlag) {
            flapState = FlapState.NONE;
            return;
        }

        if (rightLimitSwitch.get()) {
            rightFlap.set(-.05);
        } else {
            rFlapEncoder.setPosition(0);
            rightFlap.set(0);
            rightHomeFlag = true;
        }

        if (leftLimitSwitch.get()) {
            leftFlap.set(-.05);
        } else {
            lFlapEncoder.setPosition(0);
            leftFlap.set(0);
            leftHomeFlag = true;
        }
    }

    public void setShooter(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
    }

    public boolean shouldShoot() {
        if(shooterState.equals(ShooterState.STOP))
            return false;
        return isOnTarget();
    }

    public boolean flapsArrived(double leftPos, double rightPos) {
        double dL = Math.abs(leftPos - lFlapEncoder.getPosition()),
                dR = Math.abs(rightPos - rFlapEncoder.getPosition());
        return dL < kFlapTolerance && dR < kFlapTolerance;
    }

    public boolean shooterPitchArrived() {
        return shooterPitchPID.atSetpoint();
    }

    public void resetFlaps() {
        leftHomeFlag = false;
        rightHomeFlag = false;
        flapState = FlapState.RESET;
    }

    public Pair<Double, Double> getFlapCalculatedAngle(Pose2d pose) {
        double flapLeftAngle = kLeft90, flapRightAngle = kRight90;
        double speakerY = 5.5;

        double deltaY = pose.getY() - speakerY;
        double hyp = Math.sqrt(deltaY * deltaY + pose.getX() * pose.getX());
        double sin = Math.sin(deltaY / hyp);
        double flapCenter = Math.toDegrees(sin - pose.getRotation().getRadians());
        SmartDashboard.putNumber("Flap Center", flapCenter);
        // Verify robot's angle is not outside the max angle the flaps should align at.
        if(!(Math.abs(flapCenter) > kMaxFlapAngle)) {
            double v = 78.0 * Math.sin(Math.toRadians(flapCenter));
            flapRightAngle = (90 + v) * rightFlapDegrees;
            flapLeftAngle = (90 - v) * leftFlapDegrees;
        }
        return new Pair<>(flapLeftAngle, flapRightAngle);
    }

    public double getShooterCalculatedAngle(Pose2d pose) {
        double speakerHeight = SmartDashboard.getNumber("speaker height", 92);
        // Get speaker pose constant.
        Pose3d speaker = new Pose3d(0.3, 5.5, Units.inchesToMeters(speakerHeight), new Rotation3d());
        // Get shooter pivot location relative to the center of the robot.
        Transform3d shooterPivot = new Transform3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(6.0), new Rotation3d());
        // Convert robot pose to shooter pivot pose relative to the field.
        Pose3d deltaPose = new Pose3d(pose).plus(shooterPivot);
        // Delta X and Delta Y variables to calculate distance from the speaker.
        double deltaX = deltaPose.getX() - speaker.getX(), deltaY = deltaPose.getY() - speaker.getY();
        // Calculate hypotenuse to get the distance from the speaker & the height of the speaker relative to the pivot.
        double b = Math.sqrt(deltaX * deltaX + deltaY * deltaY), a = speaker.getZ() - shooterPivot.getZ();
        // Calculate angle using arc-tangent.
        double angle = Math.toDegrees(Math.atan(a / b));
        SmartDashboard.putNumber("Shooter Pivot Calculated Angle", angle);
        return angle;
    }

    public double getShooterPitchCompensated() {
        return getShooterPitch() - kPitchMinimumAngle;
    }

    public double getShooterPitch() {
        return shooterPitchEncoder.getPosition();
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public FlapState getFlapState() {
        return flapState;
    }

    public void setFlapState(FlapState flapState) {
        this.flapState = flapState;
    }

    public PitchState getPitchState() {
        return pitchState;
    }

    public void setPitchState(PitchState pitchState) {
        this.pitchState = pitchState;
    }

    @Override
    public void periodic() {
        if (isOnTarget()) {
            if (rollingAverage < 10)
                rollingAverage++;
        } else if (rollingAverage > 0)
            rollingAverage--;

        SmartDashboard.putNumber("Average Velocity", getVelocity());
        SmartDashboard.putBoolean("Launcher On Target", isOnTarget());
        SmartDashboard.putNumber("Target Velocity", targetVelocity);

        SmartDashboard.putString("Shooter State", shooterState.name());
        SmartDashboard.putString("Flap State", flapState.name());
        SmartDashboard.putString("Shooter Angle State", pitchState.name());

        SmartDashboard.putNumber("Shooter Pitch", shooterPitchEncoder.getPosition());

        dashboardVerbose();

        // If limit switch is ever tripped, zeroes the encoders
        if (!leftLimitSwitch.get())
            lFlapEncoder.setPosition(0);
        if (!rightLimitSwitch.get())
            rFlapEncoder.setPosition(0);
    }

    /**
     * Update SmartDashboard with new shooter values. Includes all relevant data of the shooter.
     */
    private void dashboardVerbose() {
        SmartDashboard.putNumber("Left Velocity", leftShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", rightShooterEncoder.getVelocity());
        SmartDashboard.putBoolean("Avg Launcher On Target", isOnTargetAverage(7));

        SmartDashboard.putNumber("Left Flap Encoder", lFlapEncoder.getPosition());
        SmartDashboard.putNumber("Right Flap Encoder", rFlapEncoder.getPosition());

        SmartDashboard.putBoolean("Left Flap Limit Switch", leftLimitSwitch.get());
        SmartDashboard.putBoolean("Right Flap Limit Switch", rightLimitSwitch.get());

        SmartDashboard.putBoolean("Left Flap Home", leftHomeFlag);
        SmartDashboard.putBoolean("Right Flap Home", rightHomeFlag);

        SmartDashboard.putBoolean("Front Limit Switch", forwardLimitSwitch.get());
        SmartDashboard.putBoolean("Back Limit Switch", backwardLimitSwitch.get());

        SmartDashboard.putNumber("Shooter Pitch Comp", getShooterPitchCompensated());
    }

    /**
     * The method to retrieve the Shooter instance; creates new instance if the Shooter has not been instantiated.
     * @return Shooter instance.
     */
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            return new ShooterSubsystem();
        }
        return instance;
    }
}