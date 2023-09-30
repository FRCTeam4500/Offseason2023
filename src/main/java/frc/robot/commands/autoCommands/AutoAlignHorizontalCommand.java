package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignHorizontalCommand extends CommandBase {

	private int limelightId;
	private SwerveDrive swerve;
	private Vision vision;
	private double timeThreshold;
	private double translationThreshold;
	private int timeCorrect;
	private PIDController pid;

	public AutoAlignHorizontalCommand(
		int limelightId
	) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.limelightId = limelightId;
		timeThreshold = 0.5;
		translationThreshold = 1;
		this.pid = new PIDController(1, 0, 0);
		addRequirements(swerve, vision);
	}

	@Override
	public void initialize() {
		pid.reset();
		pid.setSetpoint(0);
		timeCorrect = 0;
		vision.getLimelight(limelightId).setPipeline(VisionConstants.REFLECTION_PIPELINE);
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
