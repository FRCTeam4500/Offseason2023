package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystem.messaging.MessagingSystem;
import frc.robot.subsystem.placer.Placer;
import frc.robot.subsystem.swerve.SwerveDrive;

public class RobotContainer {

	private final DriveController driveStick = DriveController.getInstance();

	private final OperatorController controlJoystick = OperatorController.getInstance();

	private final Autonomous autonomous = Autonomous.getInstance();

	private final Placer placer = Placer.getInstance();

	public RobotContainer() {}

	public Command getAutonomousCommand() {
		return autonomous.getAutonCommand();
	}

	public void teleopInit() {
		Command auton = autonomous.getAutonCommand();
		if (auton != null) {
			auton.cancel();
		}
	}

	public void disabledInit() {
		SwerveDrive.getInstance().zeroModules();
	}
}
