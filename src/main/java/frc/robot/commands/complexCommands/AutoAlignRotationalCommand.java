package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignRotationalCommand extends CommandBase {

	private int limelightId;
	private SwerveDrive swerve;
	private Vision vision;
	private PIDController pid;
	private int timeThreshold;
	private double rotationalThreshold;
	private int timesCorrect;

	public AutoAlignRotationalCommand(
		int limelightId,
		int timeThreshold,
		double rotationThreshold
	) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.limelightId = limelightId;
		this.pid = new PIDController(1, 0, 0);
		this.timeThreshold = timeThreshold;
		this.rotationalThreshold = rotationThreshold;
		this.timesCorrect = 0;
		addRequirements(swerve, vision);
		pid.reset();
		pid.setSetpoint(0);
	}

	@Override
	public void execute() {
		double horizontalAngleOffset = Units.radiansToDegrees(
			vision.getHorizontalAngleOffset(limelightId)
		);
		swerve.driveRobotCentric(
			0,
			0,
			pid.calculate(horizontalAngleOffset) / 10
		);
		if (Math.abs(horizontalAngleOffset) < rotationalThreshold) {
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
