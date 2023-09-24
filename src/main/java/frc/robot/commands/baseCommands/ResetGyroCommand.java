package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetGyroCommand extends CommandBase {

	private SwerveDrive swerve;
	private double offset;

	public ResetGyroCommand(double degreeOffset) {
		this.swerve = SwerveDrive.getInstance();
		this.offset = degreeOffset;
	}

	public ResetGyroCommand() {
		this.swerve = SwerveDrive.getInstance();
		offset = 0;
	}

	public void initialize() {
		swerve.resetRobotAngle(offset);
	}

	public boolean isFinished() {
		return true;
	}
}
