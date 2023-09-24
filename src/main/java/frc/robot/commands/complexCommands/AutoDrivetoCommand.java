package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoDrivetoCommand extends CommandBase {

	private int limelightId;
	private SwerveDrive swerve;
	private Vision vision;
	private PIDController pid;

	public AutoDrivetoCommand(int limelightId) {
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
		this.limelightId = limelightId;
		this.pid = new PIDController(1, 0, 0);
		addRequirements(swerve, vision);
		pid.reset();
		pid.setSetpoint(7);
	}

	@Override
	public void execute() {
		double area = vision.getTakenArea(limelightId);
		if (vision.hasValidTargets(limelightId)) {
			swerve.driveRobotCentric(pid.calculate(area) / 2, 0, 0);
		} else {
			swerve.driveRobotCentric(0, 0, 0);
		}
	}

	@Override
	public boolean isFinished() {
		return vision.getTakenArea(limelightId) > 7;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.driveRobotCentric(0, 0, 0);
	}
}
