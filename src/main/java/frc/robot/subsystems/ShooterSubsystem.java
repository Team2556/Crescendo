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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlapValues;
import frc.robot.Constants.MotorPorts;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkFlex leftShooter;
  private CANSparkFlex rightShooter;
  public CANSparkMax leftFlap;
  public CANSparkMax rightFlap;
  public RelativeEncoder lFlapEncoder;
  public RelativeEncoder rFlapEncoder;
  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;
  private SparkPIDController leftShooterPID;
  private SparkPIDController rightShooterPID;
  private SparkPIDController lFlapPID;
  private SparkPIDController rFlapPID;
  private double targetVelocity = 0;
  private int rollingAverage = 0;
  private final static ShooterSubsystem instance = ShooterSubsystem.getInstance();
  public final DigitalInput leftLimitSwitch;
  public final DigitalInput rightLimitSwitch;
  public boolean leftHomeFlag = false;
  public boolean rightHomeFlag = false;
  

  public ShooterSubsystem() {
    leftShooter = new CANSparkFlex(MotorPorts.kLeftShooterPort, MotorType.kBrushless);
    rightShooter = new CANSparkFlex(MotorPorts.kRightShooterPort, MotorType.kBrushless);
    
    leftLimitSwitch = new DigitalInput(Constants.DigitalInputs.kLeftLimitSwitch);
    rightLimitSwitch = new DigitalInput(Constants.DigitalInputs.kRightLimitSwitch);

    leftFlap = new CANSparkMax(MotorPorts.kLeftFlap, MotorType.kBrushless);
    rightFlap = new CANSparkMax(MotorPorts.kRightFlap, MotorType.kBrushless);

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

    stop();

    leftShooter.burnFlash();
    rightShooter.burnFlash();

    leftFlap.burnFlash();
    rightFlap.burnFlash();

    leftHomeFlag = false;
    rightHomeFlag = false;
  }

  //Uses PID to bring rpm of shooter to parameter
  public void setShooterVelocity(double velocity) {
    targetVelocity = velocity;
    leftShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
    rightShooterPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  //Uses PID to bring flaps to parameters
  public void setFlapPosition(double leftPosition, double rightPosition) {
    lFlapPID.setReference(leftPosition, CANSparkBase.ControlType.kPosition);
    rFlapPID.setReference(rightPosition, CANSparkBase.ControlType.kPosition);
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
  public void flapHome() {
    if (!rightHomeFlag || !leftHomeFlag) {
    
      if (!rightLimitSwitch.get()) {
        rightFlap.set(-.05);
      } else  {
        rightFlap.set(0);
        
        rightHomeFlag = true;
      }
      if (!leftLimitSwitch.get()) {
        leftFlap.set(-.05);
      } else {
        leftFlap.set(0);
        
        leftHomeFlag = true;
      }
      rFlapEncoder.setPosition(0);
      lFlapEncoder.setPosition(0);
    }
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

    SmartDashboard.putBoolean("L Home", leftHomeFlag);
    SmartDashboard.putBoolean("R Home", rightHomeFlag);

    //If limit switch is ever tripped, zeroes the encoders
    if (leftLimitSwitch.get()) {
      lFlapEncoder.setPosition(0);
    }
    if (rightLimitSwitch.get()) {
      rFlapEncoder.setPosition(0);
    }
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      return new ShooterSubsystem();
    }
    return instance;
  } 
}