// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import swervelib.SwerveController;


public class PhotonCommand extends Command {
  /** Creates a new PhotonCommand. */
  // private PhotonSubsystem m_photonSubsystem;
  private final PhotonSubsystem m_photonSubsystem = PhotonSubsystem.getInstance();
  //private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private SwerveSubsystem swerve;
  private Trigger m_x;
  // PhotonSubsystem movementObj = new PhotonSubsystem();
  SwerveSubsystem moveObj;

  public PhotonCommand(Trigger x) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_x = x;
    addRequirements(m_photonSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    if (m_x.getAsBoolean()) {
      // swerve.drive(
      //   m_photonSubsystem.xPID.calculate(m_photonSubsystem.verticalDistance(), 1),
      //   m_photonSubsystem.yPID.calculate(m_photonSubsystem.horizontalDistance(), 0),0 );
        // m_photonSubsystem.zPID.calculate(m_photonSubsystem.zAngle(), 180),
        // true);
      ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(m_photonSubsystem.verticalDistance(), m_photonSubsystem.horizontalDistance(),
                                                         new Rotation2d(m_photonSubsystem.zAngle() * Math.PI));
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
      translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);  
    }
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