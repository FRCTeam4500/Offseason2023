package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Align Parallel to April Tag
 */
public class AutoAlignParallelCommand extends CommandBase {

	private SwerveDrive swerve;
	private Vision vision;
	private PIDController pid;
	private double timeThreshold;
	private double rotationalThreshold;
	private int timeCorrect;
	private double rotationOffset;

	public AutoAlignParallelCommand() {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.pid = new PIDController(1, 0, 0);
		timeThreshold = 0.5;
		rotationalThreshold = 1;
		addRequirements(swerve, vision);
	}

	@Override
	public void initialize() {
		pid.reset();
		pid.setSetpoint(0);
		pid.setTolerance(rotationalThreshold);
		timeCorrect = 0;
		vision.setPipeline(0, 0);
		if (!vision.hasValidTargets(0)) {
			MessagingSystem
				.getInstance()
				.addMessage("No valid targets for " + getName());
			CommandScheduler.getInstance().schedule(new RumbleCommand(0.5));
			CommandScheduler.getInstance().cancel(this);
		}
		rotationOffset =
			vision.getRelativeTargetPose(0).getRotation().getDegrees();
	}

	@Override
	public void execute() {
		rotationOffset =
			pid.calculate(
				vision.getRelativeTargetPose(0).getRotation().getDegrees(),
				0
			);
		swerve.driveRobotCentric(0, 0, rotationOffset / 10);
		if (pid.atSetpoint()) {
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
