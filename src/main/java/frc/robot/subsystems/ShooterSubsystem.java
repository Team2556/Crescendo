// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimValues;
import frc.robot.Constants.FlapValues;
import frc.robot.Constants.MotorPorts;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PhotonSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkFlex leftShooter;
  private CANSparkFlex rightShooter;
  private CANSparkMax shooterAim;
  public CANSparkMax leftFlap;
  public CANSparkMax rightFlap;

  public RelativeEncoder lFlapEncoder;
  public RelativeEncoder rFlapEncoder;
  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;
  private RelativeEncoder shooterAimEncoder;

  private SparkPIDController leftShooterPID;
  private SparkPIDController rightShooterPID;
  private SparkPIDController lFlapPID;
  private SparkPIDController rFlapPID;
  private SparkPIDController shooterAimPID;
  
  // public final DigitalInput leftLimitSwitch;
  // public final DigitalInput rightLimitSwitch;
  public final DigitalInput frontLimitSwitch;
  public final DigitalInput rearLimitSwitch;

  private double targetVelocity = 0;
  private int rollingAverage = 0;
  
  public boolean leftHomeFlag = false;
  public boolean rightHomeFlag = false;
  public boolean aimHomeFlag = false;
  
  private final static ShooterSubsystem instance = ShooterSubsystem.getInstance();

  public ShooterSubsystem() {
    leftShooter = new CANSparkFlex(MotorPorts.kLeftShooterPort, MotorType.kBrushless);
    rightShooter = new CANSparkFlex(MotorPorts.kRightShooterPort, MotorType.kBrushless);

    leftFlap = new CANSparkMax(MotorPorts.kLeftFlap, MotorType.kBrushless);
    rightFlap = new CANSparkMax(MotorPorts.kRightFlap, MotorType.kBrushless);

    shooterAim = new CANSparkMax(11, MotorType.kBrushless);

    // leftLimitSwitch = new DigitalInput(Constants.DigitalInputs.kLeftLimitSwitch);
    // rightLimitSwitch = new DigitalInput(Constants.DigitalInputs.kRightLimitSwitch);
    frontLimitSwitch = new DigitalInput(Constants.DigitalInputs.kFrontLimitSwitch);
    rearLimitSwitch = new DigitalInput(Constants.DigitalInputs.kRearLimitSwitch);

    leftShooter.clearFaults();
    rightShooter.clearFaults();
    leftFlap.clearFaults();
    rightFlap.clearFaults();
    shooterAim.clearFaults();

    leftShooter.setInverted(false);
    rightShooter.setInverted(true);
    leftFlap.setInverted(false);
    rightFlap.setInverted(false);
    shooterAim.setInverted(false);

    leftShooterEncoder = leftShooter.getEncoder();
    rightShooterEncoder = rightShooter.getEncoder();

    leftShooterPID = leftShooter.getPIDController();
    rightShooterPID = rightShooter.getPIDController();

    leftShooterPID.setFeedbackDevice(leftShooterEncoder);
    rightShooterPID.setFeedbackDevice(rightShooterEncoder);

    leftShooterPID.setP(ShooterConstants.kLeftShooterP);
    leftShooterPID.setI(ShooterConstants.kLeftShooterI);
    leftShooterPID.setIZone(ShooterConstants.kLeftShooterIZone);
    leftShooterPID.setD(ShooterConstants.kLeftShooterD);
    leftShooterPID.setFF(ShooterConstants.kLeftShooterFF);
    leftShooterPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

    rightShooterPID.setP(ShooterConstants.kRightShooterP);
    rightShooterPID.setI(ShooterConstants.kRightShooterI);
    rightShooterPID.setIZone(ShooterConstants.kRightShooterIZone);
    rightShooterPID.setD(ShooterConstants.kRighthooterD);
    rightShooterPID.setFF(ShooterConstants.kRightShooterFF);
    rightShooterPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

    lFlapEncoder = leftFlap.getEncoder();
    rFlapEncoder = rightFlap.getEncoder();

    lFlapPID = leftFlap.getPIDController();
    rFlapPID = rightFlap.getPIDController();

    lFlapPID.setP(FlapValues.kLeftFlapP);
    lFlapPID.setI(FlapValues.kLeftFlapI);
    lFlapPID.setIZone(FlapValues.kLeftFlapIZone);
    lFlapPID.setD(FlapValues.kLeftFlapD);
    lFlapPID.setFF(FlapValues.kLeftFlapFF);
    lFlapPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

    rFlapPID.setP(FlapValues.kRightFlapP);
    rFlapPID.setI(FlapValues.kRightFlapI);
    rFlapPID.setIZone(FlapValues.kRightFlapIZone);
    rFlapPID.setD(FlapValues.kRightFlapD);
    rFlapPID.setFF(FlapValues.kRightFlapFF);
    rFlapPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

    shooterAimEncoder = shooterAim.getEncoder();

    shooterAimPID = shooterAim.getPIDController();

    shooterAimPID.setP(AimValues.kAimP);
    shooterAimPID.setI(AimValues.kAimI);
    shooterAimPID.setIZone(AimValues.kAimIZone);
    shooterAimPID.setD(AimValues.kAimD);
    shooterAimPID.setFF(AimValues.kAimFF);
    shooterAimPID.setOutputRange(ShooterConstants.kMinPIDOutput, ShooterConstants.kMaxPIDOutput);

    stop();

    leftShooter.burnFlash();
    rightShooter.burnFlash();

    leftFlap.burnFlash();
    rightFlap.burnFlash();

    shooterAim.burnFlash();

    leftHomeFlag = false;
    rightHomeFlag = false;
    aimHomeFlag = false;
  }

  //Uses PID to bring rpm of shooter to parameter
  public void setShooterVelocity(double velocity) {
    targetVelocity = velocity;
    leftShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
    rightShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  //Halves left velocity to make note spin counter clockwise
  public void setLeftShootVelocity(double outsideVelocity) {
    leftShooterPID.setReference(outsideVelocity / 2, CANSparkBase.ControlType.kVelocity);
    rightShooterPID.setReference(outsideVelocity, CANSparkBase.ControlType.kVelocity);
  }

  //Halves right velocity to make note spin clockwise
  public void setRightShootVelocity(double outsideVelocity) {
    leftShooterPID.setReference(outsideVelocity, CANSparkBase.ControlType.kVelocity);
    rightShooterPID.setReference(outsideVelocity / 2, CANSparkBase.ControlType.kVelocity);
  }

  // Will use a quadratic regression curve once able to be tested
  public double distanceToSpeed(double distance) {
    return distance;
  }

  //Bases speed of distance from april tags and how far each side is from 90 degrees they are, adds if past, subtracts if before
  public void setVelocityByTags(double distance, double leftPos, double rightPos) {
    double rightVelocity = distanceToSpeed(distance) + 
      ((Constants.FlapValues.kRight90 - rightPos) / Constants.FlapValues.kRight90) * distanceToSpeed(distance);
    double leftVelocity = distanceToSpeed(distance) + 
      ((Constants.FlapValues.kLeft90 - leftPos) / Constants.FlapValues.kLeft90) * distanceToSpeed(distance);
    rightShooterPID.setReference(rightVelocity, CANSparkBase.ControlType.kVelocity);
    leftShooterPID.setReference(leftVelocity, CANSparkBase.ControlType.kVelocity);
  }

  //Checks if the speeds of the flywheels are within 10% of desired value
  public boolean speedsOnTarget(double distance, double leftPos, double rightPos) {
    double leftTargetSpeed = distanceToSpeed(distance) + 
      ((Constants.FlapValues.kLeft90 - leftPos) / Constants.FlapValues.kLeft90) * distanceToSpeed(distance);
    boolean leftSpeedCheck = leftShooterEncoder.getVelocity() <= leftTargetSpeed * 1.1 || leftShooterEncoder.getVelocity() >= leftTargetSpeed * 0.9;
    double rightTargetSpeed = distanceToSpeed(distance) + 
      ((Constants.FlapValues.kRight90 - rightPos) / Constants.FlapValues.kRight90) * distanceToSpeed(distance);
    boolean rightSpeedCheck = rightShooterEncoder.getVelocity() <= rightTargetSpeed * 1.1 || rightShooterEncoder.getVelocity() >= rightTargetSpeed * 0.9;
    return leftSpeedCheck && rightSpeedCheck;
  }

  //Moves speeds off of placement of flaps
  public void setAngledShoot(double averageVelocity, double leftPos, double rightPos) {
    double rightVelocity = averageVelocity + 
      ((Constants.FlapValues.kRight90 - rightPos) / Constants.FlapValues.kRight90) * averageVelocity;
    double leftVelocity = averageVelocity + 
      ((Constants.FlapValues.kLeft90 - leftPos) / Constants.FlapValues.kLeft90) * averageVelocity;
    rightShooterPID.setReference(rightVelocity, CANSparkBase.ControlType.kVelocity);
    leftShooterPID.setReference(leftVelocity, CANSparkBase.ControlType.kVelocity);
  }

  //Uses PID to bring flaps to parameters
  public void setFlapPosition(double leftPosition, double rightPosition) {
    lFlapPID.setReference(leftPosition, CANSparkBase.ControlType.kPosition);
    rFlapPID.setReference(rightPosition, CANSparkBase.ControlType.kPosition);
  }
  //Will use a scale factor 
  public double angleToLeftFlapTick(double angle) {
    return angle * Constants.FlapValues.kLeft90 / 90;
  }

  public double angleToRightFlapTick(double angle) {
    return angle * Constants.FlapValues.kRight90 / 90;
  }

  //Moves flaps based on angle from april tag
  public void setFlapPositionByTags(double angle) {
    lFlapPID.setReference(angleToLeftFlapTick(angle), CANSparkBase.ControlType.kPosition);
    rFlapPID.setReference(angleToRightFlapTick(angle), CANSparkBase.ControlType.kPosition);
  }

  //Sets speed via raw voltage
  public void setSpeed(double speed) {
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  //Zeros shooter rpm
  public void stop() {
    setSpeed(0);
  }

  public double getVelocity() {
    double average = (leftShooterEncoder.getVelocity() + rightShooterEncoder.getVelocity()) / 2;
    return average;
  }
  
  //Checks if the shooter is roughly running at desired speed
  public boolean isOnTarget() {
    boolean leftOnTarget = Math.abs(targetVelocity - leftShooterEncoder.getVelocity()) <= ShooterConstants.kVelocityTolerance;
    boolean rightOnTarget = Math.abs(targetVelocity - rightShooterEncoder.getVelocity()) <= ShooterConstants.kVelocityTolerance;
    return leftOnTarget && rightOnTarget;
  }

  //Checks if it's close to target on average
  public boolean isOnTargetAverage(int percent) {
    if (percent > 10) {
      percent = 10;
    } else if (percent < 0) {
      percent = 0;
    }

    if (rollingAverage > percent) {
      return true;
    }
    return false;
  }

  //Brings flaps to 0 and zeroes the encoders
  // public void flapHome() {
  //   if (!rightHomeFlag || !leftHomeFlag) {
    
  //     if (!rightLimitSwitch.get()) {
  //       rightFlap.set(-.05);
  //     } else  {
  //       rightFlap.set(0);
        
  //       rightHomeFlag = true;
  //     }
  //     if (!leftLimitSwitch.get()) {
  //       leftFlap.set(-.05);
  //     } else {
  //       leftFlap.set(0);
        
  //       leftHomeFlag = true;
  //     }
  //     rFlapEncoder.setPosition(0);
  //     lFlapEncoder.setPosition(0);
  //   }
  // }


  public void setAimPosition(double shooterAngle) {
    shooterAimPID.setReference(shooterAngle, CANSparkBase.ControlType.kPosition);
  }

  //Will be tested to determine scale factor
  public double angleToAimTicks(double angle) {
    return 360 - angle;
  }

  //Uses vertical angle to april tag to determine angle
  public void setAimPositionByTags(double angle) {
    shooterAimPID.setReference(angleToAimTicks(angle), CANSparkBase.ControlType.kPosition);
  }

  //Moves flaps to 90 degrees and then moves aim to amp position
  public void goToAmpPosition() {
    setFlapPosition(Constants.FlapValues.kLeft90, Constants.FlapValues.kRight90);
    setAimPosition(Constants.AimValues.kAmpAim);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isOnTarget()) {
      if (rollingAverage < 10) {
        rollingAverage++;
      }
    } else if (rollingAverage > 0) {
      rollingAverage--;
    }

    SmartDashboard.putNumber("Left Velocity", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Average Velocity", getVelocity());
    SmartDashboard.putBoolean("Launcher On Target", isOnTarget());
    SmartDashboard.putBoolean("Avg Launcher On Target", isOnTargetAverage(7));
    SmartDashboard.putNumber("Target Velocity", targetVelocity);
    SmartDashboard.putNumber("Left Encoder", lFlapEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rFlapEncoder.getPosition());
    // SmartDashboard.putBoolean("Left Limit Switch", leftLimitSwitch.get());
    // SmartDashboard.putBoolean("Right Limit Switch", rightLimitSwitch.get());
    SmartDashboard.putBoolean("Front Limit Switch", frontLimitSwitch.get());
    SmartDashboard.putBoolean("Rear Limit Switch", rearLimitSwitch.get());

    SmartDashboard.putBoolean("L Home", leftHomeFlag);
    SmartDashboard.putBoolean("R Home", rightHomeFlag);

    //If limit switch is ever tripped, zeroes the encoders
    // if (leftLimitSwitch.get()) {
    //   lFlapEncoder.setPosition(0);
    // }
    // if (rightLimitSwitch.get()) {
    //   rFlapEncoder.setPosition(0);
    // }
    if (!frontLimitSwitch.get() || !rearLimitSwitch.get()) {
      shooterAim.set(0.0);
      shooterAimPID.setReference(shooterAimEncoder.getPosition(), CANSparkBase.ControlType.kPosition);
    }
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      return new ShooterSubsystem();
    }
    return instance;
  } 
}