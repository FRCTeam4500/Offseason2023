package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignHorizontalCommand extends CommandBase {
	private SwerveDrive swerve;
	private Vision vision;
	private double timeThreshold;
	private double translationThreshold;
	private int timeCorrect;
	private PIDController pid;
	private double horizontalAngleOffset;
	private VisionTarget target;

	/** Moves horizontally to line up with a vision target */
	public AutoAlignHorizontalCommand(
		VisionTarget targetType
	) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		target = targetType;
		timeThreshold = 0.5;
		translationThreshold = 1;
		this.pid = new PIDController(1, 0, 0);
		addRequirements(swerve, vision);
	}
	
	@Override
	public void initialize() {
		pid.reset();
		pid.setSetpoint(target.setpoint);
		pid.setTolerance(translationThreshold);
		timeCorrect = 0;
		vision.setPipeline(target.limelightId, target.pipeline);
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
			pid.calculate(horizontalAngleOffset) / 10,
			0
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

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Limelight Used", () -> vision.getLimelight(target.limelightId).getName(), null);
		builder.addDoubleProperty("Time Correct (Seconds)", () -> timeCorrect / 50., null);
	}
}
