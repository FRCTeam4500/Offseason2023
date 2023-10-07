package frc.robot.commands.autoCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoTimedDriveCommand extends CommandBase{
    private SwerveDrive swerve;
    private double forwardSpeed;
    private double sidewaysSpeed;
    private PIDController turningPID;
    private double seconds;
    private double endTime;
    public AutoTimedDriveCommand(double forwardVelocity, double sidewaysVelocity, double targetAngle, double timeSeconds) {
        this.swerve = SwerveDrive.getInstance();
        forwardSpeed = forwardVelocity;
        sidewaysSpeed = sidewaysVelocity;
        turningPID = new PIDController(1, 0, 0);
		turningPID.enableContinuousInput(-Math.PI, Math.PI);
        turningPID.setSetpoint(Math.toRadians(targetAngle));
        seconds = timeSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + seconds;
        turningPID.reset();
    }

    @Override
    public void execute() {
        swerve.driveFieldCentric(forwardSpeed, sidewaysSpeed, 3.5 * turningPID.calculate(swerve.getGyro().getRotation2d().getRadians()));
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