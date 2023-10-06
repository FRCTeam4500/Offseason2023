package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {

	private final Autonomous autonomous;
	private final MessagingSystem messaging;

	public RobotContainer() {
		DriveController.getInstance();
		OperatorController.getInstance();
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
		Intake.setGamePiece(GamePiece.Cone);
		SwerveDrive.getInstance().resetPose(autonomous.getStartingPosition());
	}

	public void autonomousExit() {
		SwerveDrive.getInstance().lockMovement(Math.PI / 4);
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
