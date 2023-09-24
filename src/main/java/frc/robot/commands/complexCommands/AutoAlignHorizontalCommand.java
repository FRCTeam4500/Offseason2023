package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignHorizontalCommand extends CommandBase {

	private int limelightId;
	private SwerveDrive swerve;
	private Vision vision;
	private int timeThreshold;
	private double translationThreshold;
	private int timeCorrect;
	private PIDController pid;

	public AutoAlignHorizontalCommand(
		int limelightId,
		int timeThreshold,
		double translationThreshold
	) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.limelightId = limelightId;
		this.timeThreshold = timeThreshold;
		this.timeCorrect = 0;
		this.translationThreshold = translationThreshold;
		this.pid = new PIDController(1, 0, 0);
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
			pid.calculate(horizontalAngleOffset) / 10,
			0
		);
		if (Math.abs(horizontalAngleOffset) < translationThreshold) {
			timeCorrect++;
		} else {
			timeCorrect = 0;
		}
	}

	@Override
	public boolean isFinished() {
		return timeCorrect >= timeThreshold * 50;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.driveRobotCentric(0, 0, 0);
	}
}
