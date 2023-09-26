package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.DriveController;

public class RumbleCommand extends CommandBase {
	private CommandGenericHID controller;
	private double rumbleTime;
	private double localTime;

	public RumbleCommand(double rumbleTimeSeconds) {
		controller = DriveController.getInstance();
		controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
		rumbleTime = 50 * rumbleTimeSeconds;
		localTime = 0;
	}

	@Override
	public void execute() {
		localTime++;
	}

	@Override
	public boolean isFinished() {
		return localTime >= rumbleTime;
	}

	@Override
	public void end(boolean interrupted) {
		controller.getHID().setRumble(RumbleType.kBothRumble, 0);
	}
}
