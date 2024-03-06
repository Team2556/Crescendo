package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;

/**
 * Unnecessary command to run the pose subsystem.
 * Here in-case any additional functionality is needed.
 */
public class PoseUpdateCommand extends Command {
    private final PoseSubsystem poseSubsystem;

    public PoseUpdateCommand(PoseSubsystem poseSubsystem) {
        this.poseSubsystem = poseSubsystem;

        addRequirements(poseSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}