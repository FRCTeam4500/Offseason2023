package frc.robot.commands.debugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Arm;

public class TiltArmCommand extends CommandBase {

	private Arm arm;
	private double tiltChange;

	public TiltArmCommand(Arm arm, double tiltChange) {
		this.arm = arm;
		this.tiltChange = tiltChange;
	}

	public void initialize() {
		arm.changeTilt(tiltChange);
	}

	public boolean isFinished() {
		return true;
	}
}
