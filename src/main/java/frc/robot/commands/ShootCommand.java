// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.FlapValues;
import frc.robot.Constants.ShooterConstants;

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final ShooterSubsystem m_shooterSubsystem;
  Trigger m_start;
  Trigger m_left;
  Trigger m_right;
  Trigger m_up;
  Trigger m_down;
  Trigger m_x;
  Trigger m_a;
  Trigger m_b;
  Trigger m_rBumper;
  Trigger m_lBumper;
  private final Trigger m_rightTrigger;
  Trigger m_leftTrigger;
  private final DoubleSupplier m_leftY;
  private double targetInput;
  private double aimerSpot;
  private double lFlapEncoder;
  private double rFlapEncoder;
  Timer shooterTimer = new Timer();
  Timer intakeTimer = new Timer();

  private static ShooterSpeeds shooterSpeeds = ShooterSpeeds.SPEAKER_MID;
  public enum ShooterSpeeds {
    AMP,
    SPEAKER_CLOSE,
    SPEAKER_MID,
    SPEAKER_FAR
  }

  private static FlapPositions flapPosition = FlapPositions.RESET;
  public enum FlapPositions {
    RESET,
    INTAKE,
    STRAIGHT,
    LEFT,
    RIGHT
  }
  public ShootCommand(ShooterSubsystem shooterSubsystem, Trigger trigger, Trigger trigger2, Trigger trigger3, Trigger trigger4, Trigger trigger5, DoubleSupplier leftY, Trigger trigger6, Trigger trigger7, Trigger trigger8, Trigger trigger9, Trigger trigger10, Trigger trigger11, Trigger trigger12) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    // lFlapEncoder = m_shooterSubsystem.getlFlapEncoderValue();
    m_start = trigger;
    m_left = trigger2;
    m_right = trigger3;
    m_up = trigger4;
    m_down = trigger5;
    m_leftY = leftY;
    m_leftTrigger = trigger6;
    m_rightTrigger = trigger7;
    m_x = trigger8;
    m_a = trigger9;
    m_b = trigger10;
    m_rBumper = trigger11;
    m_lBumper = trigger12;
    addRequirements(shooterSubsystem);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Flap Encoder Value", (lFlapEncoder));
    SmartDashboard.putNumber("Shooter Target Input Speed", 0);
    SmartDashboard.putNumber("Aimer Placement", 0);
    shooterSpeeds = ShooterSpeeds.AMP;
    flapPosition = FlapPositions.RESET;

    //Flags for homing
    m_shooterSubsystem.rightHomeFlag = false;
    m_shooterSubsystem.leftHomeFlag = false;
    // m_shooterSubsystem.aimFlag = false;
    // m_shooterSubsystem.flapHome();
    //SmartDashboard.putNumber("Init Left Encoder", m_shooterSubsystem.leftFlap.getEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetInput = SmartDashboard.getNumber("Shooter Target Input Speed", 0);
    aimerSpot = SmartDashboard.getNumber("Aimer Placement", 0);
    // SmartDashboard.putBoolean("y", m_y.getAsBoolean());

    //Homes flaps and aimer upon initialization
    // m_shooterSubsystem.flapHome();
    // flapPosition = FlapPositions.RESET;
    // m_shooterSubsystem.aimHome();

    if(m_rBumper.getAsBoolean()) {
      m_shooterSubsystem.setFlapPositionByTags(PhotonSubsystem.targetRotation);
      m_shooterSubsystem.setVelocityByTags(PhotonSubsystem.targetX, 
                                           m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                           m_shooterSubsystem.rFlapEncoder.getPosition());
      m_shooterSubsystem.setAimPositionByTags(PhotonSubsystem.shootAngle);
      if (!m_rBumper.getAsBoolean() || m_shooterSubsystem.speedsOnTarget(PhotonSubsystem.targetX, 
                                                                         m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                                                         m_shooterSubsystem.rFlapEncoder.getPosition())) {
        intakeTimer.reset();
        intakeTimer.start();
        if (intakeTimer.get() < 1) {
          IntakeSubsystem.setIntakeMotor(0.8);
        } else {
          IntakeSubsystem.setIntakeMotor(0.0);
        }
      }
    }

    if (m_lBumper.getAsBoolean()) {
      m_shooterSubsystem.goToAmpPosition();
    }

    if (m_a.getAsBoolean()) {
      m_shooterSubsystem.setAimPosition(aimerSpot);
    }

    if (m_b.getAsBoolean()) {
      m_shooterSubsystem.setVelocityByTags(PhotonSubsystem.targetX, 
                                           m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                           m_shooterSubsystem.rFlapEncoder.getPosition());
    }

    if (m_x.getAsBoolean()) {
      m_shooterSubsystem.setAimPositionByTags(PhotonSubsystem.shootAngle);
    }

    //If start pressed, homes and zeros flaps
    if (m_start.getAsBoolean()) {
      m_shooterSubsystem.rightHomeFlag = false;
      m_shooterSubsystem.leftHomeFlag = false;
      // m_shooterSubsystem.flapHome();
      flapPosition = FlapPositions.RESET;
    }

    //D-pads move flaps to intake, shoot, left aim, and right aim
    if (m_up.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kLeft90, FlapValues.kRight90);
      flapPosition = FlapPositions.STRAIGHT;
    }

    if (m_down.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kLeft45, FlapValues.kRight45);
      flapPosition = FlapPositions.INTAKE;
    }

    if (m_left.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kSlantLLeft, FlapValues.kSlantLRight);
      flapPosition = FlapPositions.LEFT;
    }

    if (m_right.getAsBoolean()) {
      m_shooterSubsystem.setFlapPosition(FlapValues.kSlantRLeft, FlapValues.kSlantRRight);
      flapPosition = FlapPositions.RIGHT;
    }

    switch (shooterSpeeds) {
      case AMP:
        if (m_rightTrigger.getAsBoolean()) {
          // m_shooterSubsystem.setAngledShoot(Constants.ShooterConstants.kAmpSpeed, 
          //                                   m_shooterSubsystem.lFlapEncoder.getPosition(), 
          //                                   m_shooterSubsystem.rFlapEncoder.getPosition());
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kAmpSpeed);
          m_shooterSubsystem.setSpeed(.10);
          
        } else if (m_leftTrigger.getAsBoolean()) {
          m_shooterSubsystem.setShooterVelocity(Constants.ShooterConstants.kIntakeSpeed);
        } else {
          m_shooterSubsystem.stop();
        }
        if (m_leftY.getAsDouble() < -.85) {
          shooterSpeeds = ShooterSpeeds.SPEAKER_CLOSE;
        }
        break;

      case SPEAKER_CLOSE:
        if (m_rightTrigger.getAsBoolean()) {
          m_shooterSubsystem.setAngledShoot(Constants.ShooterConstants.kSpeakerCloseSpeed, 
                                            m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                            m_shooterSubsystem.rFlapEncoder.getPosition());
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
          m_shooterSubsystem.setAngledShoot(Constants.ShooterConstants.kSpeakerMidSpeed, 
                                            m_shooterSubsystem.lFlapEncoder.getPosition(), 
                                            m_shooterSubsystem.rFlapEncoder.getPosition());
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
          // m_shooterSubsystem.setAngledShoot(Constants.ShooterConstants.kSpeakerFarSpeed, 
          //                                   m_shooterSubsystem.lFlapEncoder.getPosition(), 
          //                                   m_shooterSubsystem.rFlapEncoder.getPosition());
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