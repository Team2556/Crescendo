// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
  private BooleanSupplier xbox2_A, xbox2_B, xbox2_Y;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(BooleanSupplier A, BooleanSupplier B, BooleanSupplier Y) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);

    this.xbox2_A = A;
    this.xbox2_B = B;
    this.xbox2_Y = Y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.disableArms();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.xbox2_B.getAsBoolean()) {
      climbSubsystem.disableArms();
    }
    else if (this.xbox2_Y.getAsBoolean()) {
      climbSubsystem.extendArms();
    }
    else if (this.xbox2_A.getAsBoolean()) {
      climbSubsystem.retractArms();
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
