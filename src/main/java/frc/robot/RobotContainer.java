package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.autos.BalanceAuto;
import frc.robot.autonomous.autos.OnePieceAuto;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {

	private final Autonomous autonomous;
	private final MessagingSystem messaging;
	private static RobotContainer instance = null;
	private Command autoCommand;
	private final static SendableChooser<ArmPosition> firstPieceHeightChooser = new SendableChooser<ArmPosition>();

	private RobotContainer() {
		DriveController.getInstance();
		OperatorController.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
		firstPieceHeightChooser.setDefaultOption("Top", ArmPosition.Top);
		firstPieceHeightChooser.addOption("Middle", ArmPosition.Mid);
		Shuffleboard.getTab("Display").add("First Piece Height", firstPieceHeightChooser);
	}

	public static synchronized RobotContainer getInstance() {
		if (instance == null) {
			instance = new RobotContainer();
		} 
		return instance;
	}

	public static ArmPosition getFirstPieceHeight() {
		return firstPieceHeightChooser.getSelected();
	}

	public void autonomousInit() {
		messaging.enableMessaging();
		messaging.addMessage("Auto Started");
		switch (autonomous.getAutonCommand()) {
			case "One Piece":
				autoCommand = new OnePieceAuto();
				break;
			case "Balance": 
				autoCommand = new BalanceAuto();
				break;
			default:
				autoCommand = null;
				messaging.addMessage("No Auto Command Selected");
		}
		Intake.setGamePiece(GamePiece.Cone);
		SwerveDrive.getInstance().resetPose(autonomous.getStartingPosition());
	}

	public void autonomousExit() {
		SwerveDrive.getInstance().lockMovement(Math.PI / 4);
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
