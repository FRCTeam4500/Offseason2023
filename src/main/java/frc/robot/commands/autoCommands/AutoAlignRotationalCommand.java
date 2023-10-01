package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignRotationalCommand extends CommandBase {
	private SwerveDrive swerve;
	private Vision vision;
	private PIDController pid;
	private double timeThreshold;
	private double rotationalThreshold;
	private int timeCorrect;
	private double horizontalAngleOffset;
	private VisionTarget target;

	/** Rotates to line up with a vision target */
	public AutoAlignRotationalCommand(VisionTarget targetType
	) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.pid = new PIDController(1, 0, 0);
		this.target = targetType;
		timeThreshold = 0.5;
		rotationalThreshold = 1;
		addRequirements(swerve, vision);
	}

	@Override
	public void initialize() {
		pid.reset();
		pid.setSetpoint(target.setpoint);
		pid.setTolerance(rotationalThreshold);
		timeCorrect = 0;
		vision.setPipeline(target.limelightId, target.pipeline);;
		if (!vision.hasValidTargets(target.limelightId)) { 
			MessagingSystem.getInstance().addMessage("A " + getName() + "was scheduled, but there were no valid targets!");
            CommandScheduler.getInstance().schedule(new RumbleCommand(0.5));
            CommandScheduler.getInstance().cancel(this);
		}
		horizontalAngleOffset = Units.radiansToDegrees(
			vision.getHorizontalAngleOffset(target.limelightId)
		);
	}

	@Override
	public void execute() {
		horizontalAngleOffset = Units.radiansToDegrees(
			vision.getHorizontalAngleOffset(target.limelightId)
		);
		swerve.driveRobotCentric(
			0,
			0,
			pid.calculate(horizontalAngleOffset) / 10
		);
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
