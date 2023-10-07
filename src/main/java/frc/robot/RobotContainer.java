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
	private static RobotContainer instance = null;
	private Command autoCommand;

	private RobotContainer() {
		DriveController.getInstance();
		OperatorController.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
	}

	public static synchronized RobotContainer getInstance() {
		if (instance == null) {
			instance = new RobotContainer();
		} 
		return instance;
	}

	public void autonomousInit() {
		messaging.enableMessaging();
		messaging.addMessage("Auto Started");
		Intake.setGamePiece(GamePiece.Cone);
		autoCommand = autonomous.getAutonCommand();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void autonomousExit() {
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}

	public void teleopInit() {
		messaging.enableMessaging();
		messaging.addMessage("Teleop Started");
	}

	public void disabledInit() {
		SwerveDrive.getInstance().zeroModules();
	}
}
