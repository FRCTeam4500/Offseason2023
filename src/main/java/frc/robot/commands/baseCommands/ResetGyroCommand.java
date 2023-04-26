package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;

public class ResetGyroCommand extends CommandBase{
    private OdometricSwerve swerve;
    private double offset;
    public ResetGyroCommand(OdometricSwerve swerve, double degreeOffset) {
        this.swerve = swerve;
        this.offset = degreeOffset;
    }

    public ResetGyroCommand(OdometricSwerve swerve) {
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
