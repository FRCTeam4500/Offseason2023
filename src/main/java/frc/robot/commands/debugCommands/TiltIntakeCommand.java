package frc.robot.commands.debugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.placer.intake.Intake;

public class TiltIntakeCommand extends CommandBase {

	private Intake intake;
	private double tiltChange;

	public TiltIntakeCommand(double tiltChange) {
		this.intake = Intake.getInstance();
		this.tiltChange = tiltChange;
	}

	public void initialize() {
		intake.changeAngle(tiltChange);
	}

	public boolean isFinished() {
		return true;
	}
}
