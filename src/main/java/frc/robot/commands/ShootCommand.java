// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.FlapValues;
import frc.robot.Constants.ShooterConstants;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final ShooterSubsystem m_shooterSubsystem;
  private final Trigger m_start, m_left, m_right, m_up, m_down, m_rightTrigger, m_leftTrigger;
  private final DoubleSupplier m_leftY;
  private double targetInput;
  private double lFlapEncoder;
  private double rFlapEncoder;

  private static ShooterSpeeds shooterSpeeds = ShooterSpeeds.SPEAKER_MID;
  public enum ShooterSpeeds {
    AMP,
    SPEAKER_CLOSE,
    SPEAKER_MID,
    SPEAKER_FAR
  }
  public ShootCommand(ShooterSubsystem shooterSubsystem, Trigger start, Trigger left, Trigger right, Trigger up, Trigger down, DoubleSupplier leftY, Trigger leftTrigger, Trigger rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    // lFlapEncoder = m_shooterSubsystem.getlFlapEncoderValue();
    m_start = start;
    m_left = left;
    m_right = right;
    m_up = up;
    m_down = down;
    m_leftY = leftY;
    m_leftTrigger = leftTrigger;
    m_rightTrigger = rightTrigger;
    addRequirements(shooterSubsystem);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Flap Encoder Value", (lFlapEncoder));
    SmartDashboard.putNumber("Shooter Target Input Speed", 0);
    shooterSpeeds = ShooterSpeeds.SPEAKER_MID;

    //Flags for homing
    m_shooterSubsystem.rightHomeFlag = false;
    m_shooterSubsystem.leftHomeFlag = false;
    // m_shooterSubsystem.flapHome();
    //SmartDashboard.putNumber("Init Left Encoder", m_shooterSubsystem.leftFlap.getEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetInput = SmartDashboard.getNumber("Shooter Target Input Speed", 0);
    // SmartDashboard.putBoolean("y", m_y.getAsBoolean());

    //Homes flaps upon initialization
    m_shooterSubsystem.flapHome();

    //If start pressed, homes and zeros flaps
    if (m_start.getAsBoolean()) {
      m_shooterSubsystem.rightHomeFlag = false;
      m_shooterSubsystem.leftHomeFlag = false;
      m_shooterSubsystem.flapHome();
    }

    //D-pads move flaps to intake, shoot, left aim, and right aim
    if (m_up.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kLeft90, FlapValues.kRight90);
    }

    if (m_down.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kLeft45, FlapValues.kRight45);
    }

    if (m_left.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kSlantLLeft, FlapValues.kSlantLRight);
    }

    if (m_right.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kSlantRLeft, FlapValues.kSlantRRight);
    }

    switch (shooterSpeeds) {
      case AMP:
        if (m_rightTrigger.getAsBoolean()) {
          SmartDashboard.putBoolean("Trying to Shoot", true);
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kAmpSpeed);
        } else if (m_leftTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kIntakeSpeed);
        } else {
        SmartDashboard.putBoolean("Trying to Shoot", false);
          m_shooterSubsystem.stop();
        }
        if (m_leftY.getAsDouble() < -.85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_CLOSE;
        }
        break;

      case SPEAKER_CLOSE:
        if (m_rightTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kSpeakerCloseSpeed);
        } else if (m_leftTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kIntakeSpeed);
        } else {
          m_shooterSubsystem.stop();
        }
        if (m_leftY.getAsDouble() < -.85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_MID;
        }
        if (m_leftY.getAsDouble() > .85) {
          shooterSpeeds = ShooterSpeeds.AMP;
        }
        break;

      case SPEAKER_MID:
        if (m_rightTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kSpeakerMidSpeed);
        } else if (m_leftTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kIntakeSpeed);
        } else {
          m_shooterSubsystem.stop();
        }
        if (m_leftY.getAsDouble() < -.85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_FAR;
        }
        if (m_leftY.getAsDouble() > .85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_CLOSE;
        }
        break;

      case SPEAKER_FAR:
        if (m_rightTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kSpeakerFarSpeed);
        } else if (m_leftTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kIntakeSpeed);
        } else {
          m_shooterSubsystem.stop();
        }
        if (m_leftY.getAsDouble() > .85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_MID;
        }
        break;

      default:
      m_shooterSubsystem.setShooterVelocity(0);
        break;
    }
    SmartDashboard.putString("Shooter Case", shooterSpeeds.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
