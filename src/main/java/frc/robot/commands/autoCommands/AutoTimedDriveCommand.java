package frc.robot.commands.autoCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoTimedDriveCommand extends CommandBase{
    private SwerveDrive swerve;
    private double forwardSpeed;
    private double sidewaysSpeed;
    private double turningSpeed;
    private double seconds;
    private double endTime;
    public AutoTimedDriveCommand(ChassisSpeeds targetVelocities, double timeSeconds) {
        this.swerve = SwerveDrive.getInstance();
        forwardSpeed = targetVelocities.vxMetersPerSecond;
        sidewaysSpeed = targetVelocities.vyMetersPerSecond;
        turningSpeed = targetVelocities.omegaRadiansPerSecond;
        seconds = timeSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + seconds;
    }

    @Override
    public void execute() {
        swerve.driveFieldCentric(forwardSpeed, sidewaysSpeed, turningSpeed);
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