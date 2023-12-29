package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Gyro;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoBalanceCommand extends CommandBase {
	private Gyro navx;
	private SwerveDrive swerve;
	private PIDController pid;
	private int timeThreshold;
	private double pitchThreshold;
	private int timesCorrect;

	public AutoBalanceCommand() {
		this.swerve = SwerveDrive.getInstance();
		this.navx = swerve.getGyro();
		this.pid = new PIDController(1, 0, 0);
		this.timeThreshold = 1;
		this.pitchThreshold = 1;
		this.timesCorrect = 0;
		addRequirements(swerve);
		pid.reset();
		pid.setSetpoint(0);
	}

	@Override
	public void initialize() {
		timesCorrect = 0;
		pid.reset();
	}

	@Override
	public void execute() {
		double pitch = Math.toDegrees(navx.getPitch());
		swerve.driveRobotCentric(-pid.calculate(pitch) / 30, 0, 0);
		if (Math.abs(pitch) < pitchThreshold) {
			timesCorrect++;
		} else {
			timesCorrect = 0;
		}
	}

	@Override
	public boolean isFinished() {
		return timesCorrect >= timeThreshold * 50;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.driveRobotCentric(0, 0, 0);
	}
}
