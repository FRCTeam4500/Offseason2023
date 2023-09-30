package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
	private final Autonomous autonomous;
	private final MessagingSystem messaging;

	public RobotContainer() {
		DriveController.getInstance();
		OperatorController.getInstance();
		Placer.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
	}

	public void autonomousInit() {
		messaging.enableMessaging();
		messaging.addMessage("Auto Started");
		if (autonomous.getAutonCommand() != null) {
			autonomous.getAutonCommand().schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.enableMessaging();
		messaging.addMessage("Teleop Started");
		Command auton = autonomous.getAutonCommand();
		if (auton != null) {
			auton.cancel();
		}
	}

	public void disabledInit() {
		SwerveDrive.getInstance().zeroModules();
	}
}
