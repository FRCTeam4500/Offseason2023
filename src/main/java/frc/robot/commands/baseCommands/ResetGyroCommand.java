package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.SwerveDrive;

public class ResetGyroCommand extends CommandBase{
    private SwerveDrive swerve;
    private double offset;
    public ResetGyroCommand(SwerveDrive swerve, double degreeOffset) {
        this.swerve = swerve;
        this.offset = degreeOffset;
    }

    public ResetGyroCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        offset = 0;
    }

    public void initialize() {
        swerve.resetRobotAngle(offset);
    }

    public boolean isFinished() {
        return true;
    }
}
