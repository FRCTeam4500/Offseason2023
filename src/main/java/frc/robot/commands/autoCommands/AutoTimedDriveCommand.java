package frc.robot.commands.autoCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoTimedDriveCommand extends CommandBase{
    private SwerveDrive swerve;
    private ChassisSpeeds targetVelocities;
    private double endTime;
    public AutoTimedDriveCommand(ChassisSpeeds targetVelocities, double timeSeconds) {
        this.swerve = SwerveDrive.getInstance();
        this.targetVelocities = targetVelocities;
        this.endTime = Timer.getFPGATimestamp() + timeSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.driveModules(targetVelocities);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotCentric(0, 0, 0);
    }
}