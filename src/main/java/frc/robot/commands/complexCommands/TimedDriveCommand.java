package frc.robot.commands.complexCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TimedDriveCommand extends CommandBase{
    private ChassisSpeeds targetChassisSpeeds;
    private double timeSeconds;
    private double endTime;
    private SwerveDrive swerve;
    public TimedDriveCommand(ChassisSpeeds targetChassisSpeeds, double timeSeconds) {
        swerve = SwerveDrive.getInstance();
        this.targetChassisSpeeds = targetChassisSpeeds;
        this.timeSeconds = timeSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + timeSeconds;
    }

    @Override
    public void execute() {
        swerve.driveModules(targetChassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return endTime < Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotCentric(0, 0, 0);
    }
}
