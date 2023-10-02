package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swervydwervy.Swerve;

public class AutoTurnCommand extends CommandBase {

	private Swerve swerve;
	private PIDController pid;
	private int timeCorrect;

	public AutoTurnCommand(double targetAngle) {
		swerve = Swerve.getInstance();
		pid = new PIDController(targetAngle, targetAngle, targetAngle);
		pid.setSetpoint(Units.degreesToRadians(targetAngle));
		pid.setTolerance(1);
		pid.enableContinuousInput(-Math.PI, Math.PI);
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		pid.reset();
		timeCorrect = 0;
	}

	@Override
	public void execute() {
		double turnSpeed =
			1.5 * pid.calculate(swerve.getHeading().getRadians());
		swerve.move(0, 0, turnSpeed, false);
		if (pid.atSetpoint()) {
			timeCorrect++;
		} else {
			timeCorrect = 0;
		}
	}

	@Override
	public boolean isFinished() {
		return timeCorrect > 50;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.move(0, 0, 0, false);
	}
}
