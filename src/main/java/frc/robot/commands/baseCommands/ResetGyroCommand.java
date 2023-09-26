package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetGyroCommand extends CommandBase {
	public ResetGyroCommand(double degreeOffset) {
		SwerveDrive.getInstance().resetRobotAngle(degreeOffset);
	}

	public ResetGyroCommand() {
		SwerveDrive.getInstance().resetRobotAngle(0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
