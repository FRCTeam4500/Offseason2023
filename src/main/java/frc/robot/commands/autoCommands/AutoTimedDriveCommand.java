package frc.robot.commands.autoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swervydwervy.Swerve;

public class AutoTimedDriveCommand extends CommandBase {

	private Swerve swerve;
	private double forwardSpeed;
	private double sidewaysSpeed;
	private double turningSpeed;
	private double seconds;
	private double endTime;

	public AutoTimedDriveCommand(
		ChassisSpeeds targetVelocities,
		double timeSeconds
	) {
		this.swerve = Swerve.getInstance();
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
		swerve.move(forwardSpeed, sidewaysSpeed, turningSpeed, true);
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= endTime;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.move(0, 0, 0, false);
	}
}
