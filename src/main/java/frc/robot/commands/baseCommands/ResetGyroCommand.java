package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetGyroCommand extends CommandBase {

	private double offset;

	public ResetGyroCommand(double degreeOffset) {
		offset = degreeOffset;
	}

	public ResetGyroCommand() {
		offset = 0;
	}

	@Override
	public void initialize() {
		SwerveDrive.getInstance().resetRobotAngle(offset);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
