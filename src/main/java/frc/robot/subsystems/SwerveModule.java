// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  // private TalonFX            driveMotor;
  // private CANSparkMax        steerMotor;
  // private CANcoder           absoluteEncoder;
  // private PIDController      drivingPIDController;
  // private SparkPIDController turningPIDController;
  // private RelativeEncoder    driveEncoder;
  // private RelativeEncoder    steerEncoder;

  public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {
    // driveMotor = new TalonFX(driveMotorCANID);
    // steerMotor = new CANSparkMax(steerMotorCANID, MotorType.kBrushless);
    // absoluteEncoder = new CANcoder(cancoderCANID);

    // turningPIDController = steerMotor.getPIDController();
    
    // steerEncoder = steerMotor.getEncoder();

    // driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    // steerMotor.restoreFactoryDefaults();
    // absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    // CANcoderConfigurator cfg = encoder.getConfigurator();
    //     cfg.apply(new CANcoderConfiguration());
    //     MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
    //     cfg.refresh(magnetSensorConfiguration);
    //     cfg.apply(magnetSensorConfiguration
    //               .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
    //               .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    // steerMotor.setInverted(false);
    // turningPIDController.setFeedbackDevice(steerEncoder);

    // steerEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.kTurningEncoderPositionFactor);
    // steerEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.kTurningEncoderVelocityFactor);

    // turningPIDController.setP(Constants.SwerveModuleConstants.kTurningP);
    // turningPIDController.setI(Constants.SwerveModuleConstants.kTurningI);
    // turningPIDController.setD(Constants.SwerveModuleConstants.kTurningD);
    // turningPIDController.setFF(Constants.SwerveModuleConstants.kTurningFF);

    // driveMotor.setInverted(false);

    // drivingPIDController.setP(Constants.SwerveModuleConstants.kDrivingP);
    // drivingPIDController.setI(Constants.SwerveModuleConstants.kDrivingI);
    // drivingPIDController.setD(Constants.SwerveModuleConstants.kDrivingD);

    // steerMotor.burnFlash();

    // driveMotor.setPosition(0);
    // steerEncoder.setPosition(0);
  }

  // public double getDistance() {
  //   return driveMotor.getPosition().getValueAsDouble();
  // }

  // public Rotation2d getAngle() {
  //   return Rotation2d.fromDegrees(steerEncoder.getPosition());
  // }

  // public void setState(SwerveModuleState state) {
  //   turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
  //   drivingPIDController.calculate(driveMotor.getVelocity().getValueAsDouble(), state.speedMetersPerSecond);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
